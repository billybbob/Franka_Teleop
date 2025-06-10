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

controller_interface::return_type CartesianPoseExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) {
    // Get initial orientation and translation
    std::tie(orientation_, position_) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    initial_robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = 0.0;

    // Initialiser les commandes avec la pose actuelle
    {
      std::lock_guard<std::mutex> lock(cmd_pose_mutex_);
      target_orientation_ = orientation_;
      target_position_ = position_;
    }

    initialization_flag_ = false;
  } else {
    robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = robot_time_ - initial_robot_time_;
  }

  // Récupération thread-safe des dernières commandes de pose
  Eigen::Quaterniond new_orientation;
  Eigen::Vector3d new_position;
  
  {
    std::lock_guard<std::mutex> lock(cmd_pose_mutex_);
    new_orientation = target_orientation_;
    new_position = target_position_;
  }

  if (franka_cartesian_pose_->setCommand(new_orientation, new_position)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

void CartesianPoseExampleController::cmd_pose_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  
  // Vérification de la taille du message (7 éléments : 3 position + 4 quaternion + pince)
  if (msg->data.size() != 8) {
    RCLCPP_WARN(get_node()->get_logger(), 
                "Received cmd_pose with incorrect size: %zu, expected 7 (x,y,z,qx,qy,qz,qw)", 
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

  RCLCPP_INFO(get_node()->get_logger(), "Cartesian pose controller activated");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  
  RCLCPP_INFO(get_node()->get_logger(), "Cartesian pose controller deactivated");
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerInterface)
