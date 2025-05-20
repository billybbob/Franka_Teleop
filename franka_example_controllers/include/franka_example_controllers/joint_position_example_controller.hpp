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

#pragma once

#include <string>
#include <array>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The joint position example controller
 */
class JointPositionExampleController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  std::string robot_description_;

  std::mutex cmd_mutex_;
  std_msgs::msg::Float32MultiArray::SharedPtr last_joint_positions_;

  bool is_gazebo_{false};
  bool use_external_targets_{false};
  bool initialization_flag_{true};
  const int num_joints{7};
  
  std::vector<double> initial_q_;
  std::vector<double> target_positions_;
  
  double initial_robot_time_{0.0};
  double robot_time_{0.0};
  double elapsed_time_{0.0};
  double trajectory_period_{0.001};  // Assuming a default value
  
  // Modifié pour utiliser le bon type compatible avec le code source
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_command_subscriber_;
  
  // Fonction de callback pour traiter les messages entrants
  void commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
};

}  // namespace franka_example_controllers
