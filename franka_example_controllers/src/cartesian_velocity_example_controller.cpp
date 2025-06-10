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

#include <franka_example_controllers/cartesian_velocity_example_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <mutex>

#include <Eigen/Eigen>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_velocity_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type CartesianVelocityExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  
  // Récupération thread-safe des dernières commandes de vitesse
  Eigen::Vector3d cartesian_linear_velocity;
  Eigen::Vector3d cartesian_angular_velocity;
  
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    cartesian_linear_velocity = current_linear_velocity_;
    cartesian_angular_velocity = current_angular_velocity_;
  }

  if (franka_cartesian_velocity_->setCommand(cartesian_linear_velocity,
                                             cartesian_angular_velocity)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

void CartesianVelocityExampleController::cmd_vel_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  
  // Vérification de la taille du message (6 éléments : 3 linéaires + 3 angulaires + pince)
  if (msg->data.size() != 7) {
    RCLCPP_WARN(get_node()->get_logger(), 
                "Received cmd_vel with incorrect size: %zu, expected 7", 
                msg->data.size());
    return;
  }

  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  
  // Extraction des vitesses linéaires (x, y, z)
  current_linear_velocity_(0) = msg->data[0];  // v_x
  current_linear_velocity_(1) = msg->data[1];  // v_y
  current_linear_velocity_(2) = msg->data[2];  // v_z
  
  // Extraction des vitesses angulaires (rx, ry, rz)
  current_angular_velocity_(0) = msg->data[3];  // omega_x
  current_angular_velocity_(1) = msg->data[4];  // omega_y
  current_angular_velocity_(2) = msg->data[5];  // omega_z

  RCLCPP_DEBUG(get_node()->get_logger(), 
               "Received cmd_vel: linear=[%.3f, %.3f, %.3f], angular=[%.3f, %.3f, %.3f]",
               current_linear_velocity_(0), current_linear_velocity_(1), current_linear_velocity_(2),
               current_angular_velocity_(0), current_angular_velocity_(1), current_angular_velocity_(2));
}

controller_interface::CallbackReturn CartesianVelocityExampleController::on_init() {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
          franka_semantic_components::FrankaCartesianVelocityInterface(k_elbow_activated_));

  // Création du subscriber pour /cmd_vel
  cmd_vel_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/cmd_vel",
      10,
      std::bind(&CartesianVelocityExampleController::cmd_vel_callback, this, std::placeholders::_1));

  // Initialisation des vitesses à zéro
  current_linear_velocity_.setZero();
  current_angular_velocity_.setZero();

  // Configuration du comportement de collision de manière sécurisée
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

  RCLCPP_INFO(get_node()->get_logger(), "Cartesian velocity controller configured. Listening to /cmd_vel");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);
  
  // Réinitialisation des vitesses à l'activation
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    current_linear_velocity_.setZero();
    current_angular_velocity_.setZero();
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "Cartesian velocity controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->release_interfaces();
  
  // Arrêt du robot à la désactivation
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    current_linear_velocity_.setZero();
    current_angular_velocity_.setZero();
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "Cartesian velocity controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerInterface)
