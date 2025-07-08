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

#include <franka_example_controllers/joint_position_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <algorithm>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <mutex>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointPositionExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  
  // add the robot time interface
  if (!is_gazebo_) {
    config.names.push_back(arm_id_ + "/robot_time");
  }
  
  return config;
}

controller_interface::return_type JointPositionExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  if (initialization_flag_) {
    for (int i = 0; i < num_joints; ++i) {
      initial_q_.at(i) = state_interfaces_[i].get_value();
      target_positions_.at(i) = initial_q_.at(i);
      target_positions_raw_.at(i) = initial_q_.at(i);
      previous_positions_.at(i) = initial_q_.at(i);
      current_velocities_.at(i) = 0.0;
    }
    initialization_flag_ = false;
    if (!is_gazebo_) {
      initial_robot_time_ = state_interfaces_.back().get_value();
    }
    elapsed_time_ = 0.0;
  } else {
    if (!is_gazebo_) {
      robot_time_ = state_interfaces_.back().get_value();
      elapsed_time_ = robot_time_ - initial_robot_time_;
    } else {
      elapsed_time_ += trajectory_period_;
    }
  }

  // Calculer dt pour le lissage
  double dt = is_gazebo_ ? trajectory_period_ : 0.001; // Période de contrôle

  // Obtenir la dernière commande de position (thread-safe)
  std_msgs::msg::Float32MultiArray::SharedPtr cmd;
  bool new_cmd_received = false;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd = last_joint_positions_;
    new_cmd_received = new_command_received_;
    new_command_received_ = false;
  }

  // Si on utilise des cibles externes et qu'on a reçu une commande
  if (use_external_targets_ && cmd && !cmd->data.empty()) {
    // Mettre à jour les cibles brutes si nouvelle commande
    if (new_cmd_received) {
      for (int i = 0; i < num_joints; i++) {
        if (i < static_cast<int>(cmd->data.size())) {
          target_positions_raw_[i] = cmd->data[i];
        }
      }
    }
    
    // Appliquer le lissage de trajectoire
    smoothTrajectory(dt);
    
    // Appliquer les positions lissées
    for (int i = 0; i < num_joints; i++) {
      command_interfaces_[i].set_value(target_positions_[i]);
    }
    
    RCLCPP_DEBUG(get_node()->get_logger(), "Applied smoothed joint position commands");
  } else {
    // Si pas de commande reçue mais on utilise des cibles externes, maintenir la position actuelle
    for (int i = 0; i < num_joints; i++) {
      command_interfaces_[i].set_value(target_positions_[i]);
    }
    RCLCPP_DEBUG(get_node()->get_logger(), "Maintained current positions");
  }

  return controller_interface::return_type::OK;
}

void JointPositionExampleController::smoothTrajectory(double dt) {
  for (int i = 0; i < num_joints; ++i) {
    // Appliquer un filtre passe-bas sur la position cible
    double filtered_target = applyLowPassFilter(target_positions_raw_[i], target_positions_[i], smoothing_factor_);
    
    // Limiter la vitesse et l'accélération
    target_positions_[i] = limitVelocity(filtered_target, target_positions_[i], current_velocities_[i], dt, i);
    
    // Calculer la vitesse actuelle
    current_velocities_[i] = (target_positions_[i] - previous_positions_[i]) / dt;
    
    // Sauvegarder la position précédente
    previous_positions_[i] = target_positions_[i];
  }
}

double JointPositionExampleController::limitVelocity(double desired_pos, double current_pos, double current_vel, double dt, int joint_idx) {
  double position_error = desired_pos - current_pos;
  double max_position_change = max_velocity_ * dt;
  
  // Limiter le changement de position basé sur la vitesse maximale
  if (std::abs(position_error) > max_position_change) {
    position_error = std::copysign(max_position_change, position_error);
  }
  
  double target_velocity = position_error / dt;
  double velocity_change = target_velocity - current_vel;
  double max_velocity_change = max_acceleration_ * dt;
  
  // Limiter le changement de vitesse basé sur l'accélération maximale
  if (std::abs(velocity_change) > max_velocity_change) {
    velocity_change = std::copysign(max_velocity_change, velocity_change);
  }
  
  double limited_velocity = current_vel + velocity_change;
  return current_pos + limited_velocity * dt;
}

double JointPositionExampleController::applyLowPassFilter(double new_value, double old_value, double alpha) {
  return alpha * new_value + (1.0 - alpha) * old_value;
}

CallbackReturn JointPositionExampleController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
    auto_declare<bool>("use_external_targets", true);
    auto_declare<double>("max_velocity", 0.05);
    auto_declare<double>("max_acceleration", 0.5);
    auto_declare<double>("smoothing_factor", 0.001);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();
  use_external_targets_ = get_node()->get_parameter("use_external_targets").as_bool();
  max_velocity_ = get_node()->get_parameter("max_velocity").as_double();
  max_acceleration_ = get_node()->get_parameter("max_acceleration").as_double();
  smoothing_factor_ = get_node()->get_parameter("smoothing_factor").as_double();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  // Initialiser tous les vecteurs avec la bonne taille
  target_positions_.resize(num_joints, 0.0);
  target_positions_raw_.resize(num_joints, 0.0);
  initial_q_.resize(num_joints, 0.0);
  current_velocities_.resize(num_joints, 0.0);
  previous_positions_.resize(num_joints, 0.0);

  // Créer le subscriber pour les positions cibles
  if (use_external_targets_) {
    joint_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/joint_positions", 10, std::bind(&JointPositionExampleController::commandCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_node()->get_logger(), 
                "JointPositionExampleController initialized with external target subscription to /joint_positions");
    RCLCPP_INFO(get_node()->get_logger(), 
                "Smoothing parameters: max_vel=%.2f, max_acc=%.2f, smoothing_factor=%.2f",
                max_velocity_, max_acceleration_, smoothing_factor_);
  } else {
    RCLCPP_INFO(get_node()->get_logger(), 
                "JointPositionExampleController initialized with default trajectory behavior");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  new_command_received_ = false;
  return CallbackReturn::SUCCESS;
}

void JointPositionExampleController::commandCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  if (msg->data.size() < static_cast<size_t>(num_joints)) {
    RCLCPP_ERROR(get_node()->get_logger(), 
                "Received joint command with insufficient joint positions (%zu), expected %d",
                msg->data.size(), num_joints);
    return;
  }

  // Stocker le message avec protection mutex
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_joint_positions_ = msg;
    new_command_received_ = true;
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "Received new joint position command");
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerInterface)