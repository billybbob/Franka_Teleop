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

#include <Eigen/Dense>
#include <string>
#include <mutex>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <franka_example_controllers/robot_utils.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The cartesian pose example controller
 */
class CartesianPoseExampleController : public controller_interface::ControllerInterface {
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
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void cmd_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  
  // Fonctions utilitaires pour la limitation des vitesses
  Eigen::Vector3d limitVelocity(const Eigen::Vector3d& velocity, double max_velocity);
  Eigen::Vector3d limitAcceleration(const Eigen::Vector3d& target_velocity, 
                                   const Eigen::Vector3d& current_velocity,
                                   double max_acceleration, double dt);

  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  // Pose actuelle
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;
  Eigen::Quaterniond current_orientation_;
  Eigen::Vector3d current_position_;

  // Commandes de pose cible (thread-safe)
  Eigen::Quaterniond target_orientation_;
  Eigen::Vector3d target_position_;
  std::mutex cmd_pose_mutex_;

  // Variables de vitesse et accélération
  Eigen::Vector3d current_linear_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  Eigen::Vector3d previous_linear_velocity_;
  Eigen::Vector3d previous_angular_velocity_;
  Eigen::Vector3d target_linear_velocity_;
  Eigen::Vector3d target_angular_velocity_;

  // Subscriber pour les commandes de pose
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pose_subscriber_;

  const bool k_elbow_activated_{false};
  bool initialization_flag_{true};

  double elapsed_time_{0.0};
  double initial_robot_time_{0.0};
  double robot_time_{0.0};
  std::string robot_description_;
  std::string arm_id_;
};

}  // namespace franka_example_controllers