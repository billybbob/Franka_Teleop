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

  // Get the latest joint position command (thread-safe)
  std_msgs::msg::Float32MultiArray::SharedPtr cmd;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd = last_joint_positions_;
  }

  // If received commands and using external targets, apply them
  if (use_external_targets_ && cmd && !cmd->data.empty()) {
    for (int i = 0; i < num_joints; i++) {
      if (i < static_cast<int>(cmd->data.size())) {
        command_interfaces_[i].set_value(cmd->data[i]);
      }
    }
    RCLCPP_DEBUG(get_node()->get_logger(), "Applied external joint position commands");
  } else {
    // If no command received but using external targets, just maintain current position
    for (int i = 0; i < num_joints; i++) {
      command_interfaces_[i].set_value(target_positions_[i]);
    }
    RCLCPP_DEBUG(get_node()->get_logger(), "Maintained current positions");
  }

  return controller_interface::return_type::OK;
}

CallbackReturn JointPositionExampleController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
    auto_declare<bool>("use_external_targets", true);  // Default to using external targets
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

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  // Initialize target_positions_ with the correct size
  target_positions_.resize(num_joints, 0.0);
  initial_q_.resize(num_joints, 0.0);

  // Create subscriber for target joint positions
  if (use_external_targets_) {
    joint_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/joint_positions", 10, std::bind(&JointPositionExampleController::commandCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_node()->get_logger(), 
                "JointPositionExampleController initialized with external target subscription to /joint_positions");
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

  // Store the message with mutex protection
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_joint_positions_ = msg;
  }

  // Also update target_positions_ for fallback
  for (int i = 0; i < num_joints; ++i) {
    target_positions_.at(i) = msg->data.at(i);
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "Received new joint position command");
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerInterface)
