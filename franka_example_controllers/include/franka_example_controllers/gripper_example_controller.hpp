#pragma once

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>  // Added for Float64 messages

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * Gripper Controller with Position and Velocity Control
 *
 * This controller responds to both velocity and position commands for controlling the gripper.
 * It subscribes to:
 * - /gripper_command_velocities (std_msgs::msg::Float64): Velocity commands for the gripper
 * - /gripper_command_positions (std_msgs::msg::Float64): Position commands for the gripper
 * - /joint_states (sensor_msgs::msg::JointState): Provides current gripper joint positions
 */
class GripperExampleController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  /**
   * Callback for velocity commands received from topic
   */
  void velocityCommandCallback(const std_msgs::msg::Float64::SharedPtr msg);
  
  /**
   * Callback for position commands received from topic
   */
  void positionCommandCallback(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * Callback to process joint states
   */
  void jointstatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  
  // Parameters
  double default_velocity_{0.0};          // Default velocity
  double current_velocity_{0.0};          // Current applied velocity (with direction)
  double current_velocity_magnitude_{0.0};// Velocity magnitude (always positive)
  double max_opening_{0.04};              // Maximum gripper opening
  double min_opening_{0.0};               // Minimum gripper opening
  double position_threshold_{0.002};      // Threshold to detect when to reverse direction
  double position_gain_{5.0};             // Gain for position control
  
  // Current state
  double finger1_position_{0.0};          // Current position of finger 1
  double finger2_position_{0.0};          // Current position of finger 2
  int direction_{1};                      // Direction: 1 to open, -1 to close
  
  // Control mode and target
  int control_mode_{0};                   // Control mode: 0 for velocity, 1 for position
  double target_position_{0.0};           // Target position for position control
};

}  // namespace franka_example_controllers
