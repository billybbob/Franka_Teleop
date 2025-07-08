// Copyright (c) 2023 Franka Robotics GmbH

#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <franka_example_controllers/joint_velocity_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <mutex>

#include <Eigen/Dense>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Configurer les interfaces pour les 7 articulations principales
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }  

  return config;
}

controller_interface::InterfaceConfiguration
JointVelocityExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  return config;
}

controller_interface::return_type JointVelocityExampleController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& /*period*/) {

  std_msgs::msg::Float32MultiArray::SharedPtr cmd;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd = last_joint_velocities_;
  }

  if (cmd && !cmd->data.empty()) {
    // Appliquer les vitesses à toutes les articulations
    for (int i = 0; i < num_joints; i++) {
      if (i < static_cast<int>(cmd->data.size())) {
        command_interfaces_[i].set_value(cmd->data[i]);
      }
    }
    
  } else {
    // Si pas de commande reçue, mettre toutes les vitesses à zéro
    for (int i = 0; i < command_interfaces_.size(); i++) {
      command_interfaces_[i].set_value(0.0);
    }
  }

  return controller_interface::return_type::OK;
}

void JointVelocityExampleController::cmdVelCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  last_joint_velocities_ = msg;
}

CallbackReturn JointVelocityExampleController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo = get_node()->get_parameter("gazebo").as_bool();

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

  // Subscription to cmd_vel
  cmd_vel_sub_ = get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/joint_velocities", 10, std::bind(&JointVelocityExampleController::cmdVelCallback, this, std::placeholders::_1));

  if (!is_gazebo) {
    auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
        "service_server/set_full_collision_behavior");
    auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

    auto future_result = client->async_send_request(request);
    future_result.wait_for(robot_utils::time_out);

    auto success = future_result.get();
    if (!success) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
      return CallbackReturn::ERROR;
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerInterface)