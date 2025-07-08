// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_example_controllers/cartesian_pose_example_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <mutex>

#include <Eigen/Eigen>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace franka_example_controllers {

// Limites de sécurité pour les vitesses et accélérations (plus conservatrices)
constexpr double MAX_LINEAR_VELOCITY = 0.02;      // m/s (réduit pour plus de sécurité)
constexpr double MAX_ANGULAR_VELOCITY = 0.04;     // rad/s (réduit pour plus de sécurité)
constexpr double MAX_LINEAR_ACCELERATION = 0.1;  // m/s² (réduit pour éviter les discontinuités)
constexpr double MAX_ANGULAR_ACCELERATION = 0.1; // rad/s² (réduit pour éviter les discontinuités)
constexpr double CONTROL_FREQUENCY = 1000.0;     // Hz (fréquence du contrôleur)

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  // add the robot time interface
  config.names.push_back(arm_id_ + "/robot_time");
  return config;
}

// Fonction pour valider et limiter les vitesses
Eigen::Vector3d CartesianPoseExampleController::limitVelocity(
    const Eigen::Vector3d& velocity, double max_velocity) {
  Eigen::Vector3d limited_velocity = velocity;
  
  // Vérifier les valeurs invalides (NaN, inf)
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(limited_velocity(i))) {
      limited_velocity(i) = 0.0;
    }
  }
  
  // Limiter la norme du vecteur vitesse
  double velocity_norm = limited_velocity.norm();
  if (velocity_norm > max_velocity) {
    limited_velocity = limited_velocity * (max_velocity / velocity_norm);
  }
  
  return limited_velocity;
}

// Fonction pour limiter l'accélération (remplace le lissage)
Eigen::Vector3d CartesianPoseExampleController::limitAcceleration(
    const Eigen::Vector3d& target_velocity, 
    const Eigen::Vector3d& current_velocity,
    double max_acceleration,
    double dt) {
  
  Eigen::Vector3d velocity_diff = target_velocity - current_velocity;
  double max_velocity_change = max_acceleration * dt;
  
  // Vérifier les valeurs invalides dans la différence
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(velocity_diff(i))) {
      velocity_diff(i) = 0.0;
    }
  }
  
  // Limiter le changement de vitesse (accélération)
  double velocity_change_norm = velocity_diff.norm();
  if (velocity_change_norm > max_velocity_change) {
    velocity_diff = velocity_diff * (max_velocity_change / velocity_change_norm);
  }
  
  return current_velocity + velocity_diff;
}

controller_interface::return_type CartesianPoseExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {

  // Calcul du temps d'échantillonnage réel
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.01) {  // Sécurité : utiliser la fréquence nominale si dt invalide
    dt = 1.0 / CONTROL_FREQUENCY;
  }

  if (initialization_flag_) {
    // Get initial orientation and translation
    std::tie(orientation_, position_) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    initial_robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = 0.0;

    // Initialiser les poses actuelles et cibles avec la pose actuelle
    current_orientation_ = orientation_;
    current_position_ = position_;

    {
      std::lock_guard<std::mutex> lock(cmd_pose_mutex_);
      target_orientation_ = orientation_;
      target_position_ = position_;
    }

    // Initialiser toutes les vitesses à zéro
    current_linear_velocity_.setZero();
    current_angular_velocity_.setZero();
    target_linear_velocity_.setZero();
    target_angular_velocity_.setZero();
    previous_linear_velocity_.setZero();
    previous_angular_velocity_.setZero();

    initialization_flag_ = false;
    
    // Première commande : maintenir la pose actuelle (pas de mouvement)
    if (franka_cartesian_pose_->setCommand(orientation_, position_)) {
      return controller_interface::return_type::OK;
    } else {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "Initial set command failed. Did you activate the elbow command interface?");
      return controller_interface::return_type::ERROR;
    }
  }

  // Mise à jour du temps robot
  robot_time_ = state_interfaces_.back().get_value();
  elapsed_time_ = robot_time_ - initial_robot_time_;

  // Récupération thread-safe des dernières commandes de pose
  Eigen::Quaterniond target_orientation;
  Eigen::Vector3d target_position;
  
  {
    std::lock_guard<std::mutex> lock(cmd_pose_mutex_);
    target_orientation = target_orientation_;
    target_position = target_position_;
  }

  // 1. Calcul des vitesses souhaitées (différence entre cible et position actuelle)
  Eigen::Vector3d desired_linear_velocity = (target_position - current_position_) / dt;

  // 2. Calcul des vitesses angulaires souhaitées
  Eigen::Quaterniond orientation_diff = target_orientation * current_orientation_.conjugate();
  Eigen::Vector3d desired_angular_velocity = Eigen::Vector3d::Zero();
  
  // Normaliser et assurer la forme canonique du quaternion différence
  orientation_diff.normalize();
  if (orientation_diff.w() < 0) {
    orientation_diff.coeffs() *= -1;
  }
  
  // Conversion quaternion -> vitesse angulaire (approximation pour petits angles)
  double angle = 2.0 * std::acos(std::min(1.0, std::abs(orientation_diff.w())));
  if (angle > 1e-6) {
    Eigen::Vector3d axis(orientation_diff.x(), orientation_diff.y(), orientation_diff.z());
    double axis_norm = axis.norm();
    if (axis_norm > 1e-6) {
      axis /= axis_norm;
      desired_angular_velocity = axis * angle / dt;
    }
  }

  // 3. Application des limites de vitesse
  target_linear_velocity_ = limitVelocity(desired_linear_velocity, MAX_LINEAR_VELOCITY);
  target_angular_velocity_ = limitVelocity(desired_angular_velocity, MAX_ANGULAR_VELOCITY);
  
  // 4. Limitation d'accélération pour éviter les discontinuités
  current_linear_velocity_ = limitAcceleration(
      target_linear_velocity_, current_linear_velocity_, MAX_LINEAR_ACCELERATION, dt);
  current_angular_velocity_ = limitAcceleration(
      target_angular_velocity_, current_angular_velocity_, MAX_ANGULAR_ACCELERATION, dt);

  // 5. Intégration des vitesses pour calculer la nouvelle pose
  Eigen::Vector3d next_position = current_position_ + current_linear_velocity_ * dt;
  
  // Intégration de la vitesse angulaire pour l'orientation
  Eigen::Quaterniond next_orientation = current_orientation_;
  if (current_angular_velocity_.norm() > 1e-6) {
    double rotation_angle = current_angular_velocity_.norm() * dt;
    Eigen::Vector3d rotation_axis = current_angular_velocity_.normalized();
    Eigen::Quaterniond rotation_delta(Eigen::AngleAxisd(rotation_angle, rotation_axis));
    next_orientation = rotation_delta * current_orientation_;
  }
  next_orientation.normalize();

  // 6. Mise à jour des positions actuelles pour le prochain cycle
  current_position_ = next_position;
  current_orientation_ = next_orientation;

  // 7. Calcul des accélérations pour debug
  Eigen::Vector3d current_linear_acceleration = 
      (current_linear_velocity_ - previous_linear_velocity_) / dt;
  Eigen::Vector3d current_angular_acceleration = 
      (current_angular_velocity_ - previous_angular_velocity_) / dt;
  
  // Mise à jour des vitesses précédentes
  previous_linear_velocity_ = current_linear_velocity_;
  previous_angular_velocity_ = current_angular_velocity_;

  // Debug : affichage périodique
  static int debug_counter = 0;
  if (++debug_counter >= 1000) {  // Affichage toutes les secondes
    debug_counter = 0;
    RCLCPP_DEBUG(get_node()->get_logger(), 
                 "Vel: lin=%.3f ang=%.3f, Acc: lin=%.3f ang=%.3f",
                 current_linear_velocity_.norm(), current_angular_velocity_.norm(),
                 current_linear_acceleration.norm(), current_angular_acceleration.norm());
  }

  // 8. Envoi de la commande avec la pose calculée
  if (franka_cartesian_pose_->setCommand(next_orientation, next_position)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

void CartesianPoseExampleController::cmd_pose_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  
  // Vérification de la taille du message (7 ou 8 éléments)
  if (msg->data.size() != 7 && msg->data.size() != 8) {
    RCLCPP_WARN(get_node()->get_logger(), 
                "Received cmd_pose with incorrect size: %zu, expected 7 or 8 (x,y,z,qx,qy,qz,qw [,gripper])", 
                msg->data.size());
    return;
  }

  std::lock_guard<std::mutex> lock(cmd_pose_mutex_);
  
  // Extraction de la position (x, y, z)
  target_position_(0) = msg->data[0];  // x
  target_position_(1) = msg->data[1];  // y
  target_position_(2) = msg->data[2];  // z
  
  // Extraction du quaternion (qx, qy, qz, qw)
  target_orientation_.x() = msg->data[3];  // qx
  target_orientation_.y() = msg->data[4];  // qy
  target_orientation_.z() = msg->data[5];  // qz
  target_orientation_.w() = msg->data[6];  // qw
  
  // Normalisation du quaternion pour s'assurer qu'il est valide
  target_orientation_.normalize();

  RCLCPP_DEBUG(get_node()->get_logger(), 
               "Received cmd_pose: position=[%.3f, %.3f, %.3f], quaternion=[%.3f, %.3f, %.3f, %.3f]",
               target_position_(0), target_position_(1), target_position_(2),
               target_orientation_.x(), target_orientation_.y(), target_orientation_.z(), target_orientation_.w());
}

CallbackReturn CartesianPoseExampleController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // Création du subscriber pour /cmd_pose
  cmd_pose_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/cmd_pose",
      10,
      std::bind(&CartesianPoseExampleController::cmd_pose_callback, this, std::placeholders::_1));

  // Initialisation des vitesses à zéro
  current_linear_velocity_.setZero();
  current_angular_velocity_.setZero();
  target_linear_velocity_.setZero();
  target_angular_velocity_.setZero();
  previous_linear_velocity_.setZero();
  previous_angular_velocity_.setZero();
  
  // Configuration du comportement de collision par défaut de manière synchrone
  try {
    auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
        "service_server/set_full_collision_behavior");
    
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(get_node()->get_logger(), 
                  "Collision behavior service not available, continuing without setting it");
    } else {
      auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
      auto future_result = client->async_send_request(request);
      
      // Attendre de manière non-bloquante
      if (future_result.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        auto success = future_result.get();
        if (success) {
          RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
        } else {
          RCLCPP_WARN(get_node()->get_logger(), "Failed to set default collision behavior.");
        }
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "Timeout setting collision behavior.");
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_node()->get_logger(), 
                "Exception while setting collision behavior: %s", e.what());
  }

  // Récupération du robot_description de manière simplifiée
  try {
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
        get_node(), "/robot_state_publisher");
    
    if (parameters_client->wait_for_service(std::chrono::seconds(2))) {
      auto parameters = parameters_client->get_parameters({"robot_description"});
      if (!parameters.empty()) {
        robot_description_ = parameters[0].value_to_string();
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to get robot_description parameter.");
        robot_description_ = "";  // Valeur par défaut
      }
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "robot_state_publisher service not available.");
      robot_description_ = "";  // Valeur par défaut
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_node()->get_logger(), 
                "Exception while getting robot description: %s", e.what());
    robot_description_ = "";  // Valeur par défaut
  }

  // Récupération de l'arm_id de manière sécurisée
  if (!robot_description_.empty()) {
    arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());
  } else {
    arm_id_ = "fr3";  // Valeur par défaut
    RCLCPP_WARN(get_node()->get_logger(), "Using default arm_id: %s", arm_id_.c_str());
  }

  RCLCPP_INFO(get_node()->get_logger(), "Cartesian pose controller configured. Listening to /cmd_pose");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  // Réinitialisation des vitesses à l'activation
  {
    std::lock_guard<std::mutex> lock(cmd_pose_mutex_);
    current_linear_velocity_.setZero();
    current_angular_velocity_.setZero();
    target_linear_velocity_.setZero();
    target_angular_velocity_.setZero();
    previous_linear_velocity_.setZero();
    previous_angular_velocity_.setZero();
  }

  RCLCPP_INFO(get_node()->get_logger(), "Cartesian pose controller activated");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  
  // Arrêt du robot à la désactivation
  {
    std::lock_guard<std::mutex> lock(cmd_pose_mutex_);
    current_linear_velocity_.setZero();
    current_angular_velocity_.setZero();
    target_linear_velocity_.setZero();
    target_angular_velocity_.setZero();
    previous_linear_velocity_.setZero();
    previous_angular_velocity_.setZero();
  }

  RCLCPP_INFO(get_node()->get_logger(), "Cartesian pose controller deactivated");
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerInterface)