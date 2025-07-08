#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "fmt/format.h"
#include "franka_example_controllers/gripper_example_controller_simu.hpp"

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#define GREEN "\033[1;32m"
#define RESET "\033[0m"

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
GripperExampleControllerSimu::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  config.names.push_back("fr3_finger_joint1/velocity");
  config.names.push_back("fr3_finger_joint2/velocity");
  
  return config;
}

controller_interface::InterfaceConfiguration
GripperExampleControllerSimu::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  config.names.push_back("fr3_finger_joint1/velocity");
  config.names.push_back("fr3_finger_joint2/velocity");
  
  return config;
}

CallbackReturn GripperExampleControllerSimu::on_init() {
  try {
    auto_declare<double>("default_velocity", 0.0);       // Default velocity if none is provided
    auto_declare<double>("max_opening", 0.04);           // Maximum opening position
    auto_declare<double>("min_opening", 0.0);            // Minimum opening position
    auto_declare<double>("position_threshold", 0.002);   // Threshold to detect when to reverse
    auto_declare<double>("position_gain", 5.0);          // Gain for position control
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperExampleControllerSimu::on_configure(const rclcpp_lifecycle::State&) {
  default_velocity_ = get_node()->get_parameter("default_velocity").as_double();
  current_velocity_ = default_velocity_;
  max_opening_ = get_node()->get_parameter("max_opening").as_double();
  min_opening_ = get_node()->get_parameter("min_opening").as_double();
  position_threshold_ = get_node()->get_parameter("position_threshold").as_double();
  position_gain_ = get_node()->get_parameter("position_gain").as_double();
  
  // Initialize gripper positions
  finger1_position_ = 0.0;
  finger2_position_ = 0.0;
  
  // Initialize target position
  target_position_ = 0.0;
  
  // Control mode (0 for velocity, 1 for position)
  control_mode_ = 0;
  
  // Direction flag (1 for opening, -1 for closing)
  direction_ = 1;

  // Create subscription to gripper velocity commands
  velocity_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      "/gripper_command_velocities", 10,
      std::bind(&GripperExampleControllerSimu::velocityCommandCallback, this, std::placeholders::_1));

  // Create subscription to gripper position commands
  position_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "/gripper_command_positions", 10,
    std::bind(&GripperExampleControllerSimu::positionCommandCallback, this, std::placeholders::_1));

  // Create subscription to joint states
  joint_states_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&GripperExampleControllerSimu::jointstatesCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(get_node()->get_logger(), 
              "Gripper controller with position and velocity control configured.");
  RCLCPP_INFO(get_node()->get_logger(), 
              "Subscribed to /gripper_command_velocities, /gripper_command_positions and /joint_states");
  RCLCPP_INFO(get_node()->get_logger(), 
              "Using default velocity: %f, max_opening: %f, position_gain: %f", 
              default_velocity_, max_opening_, position_gain_);
  
  return CallbackReturn::SUCCESS;
}

void GripperExampleControllerSimu::velocityCommandCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  // Switch to velocity control mode
  control_mode_ = 0;
  
  // Update the gripper velocity magnitude with the received command
  if (msg->data > 0.0) {
    // We only update the magnitude; direction is handled automatically
    current_velocity_magnitude_ = msg->data;
    RCLCPP_DEBUG(get_node()->get_logger(), "Switched to velocity mode. Updated gripper velocity magnitude to: %f", current_velocity_magnitude_);
    
    // Update the current velocity by applying direction
    current_velocity_ = direction_ * current_velocity_magnitude_;
  } else {
    RCLCPP_DEBUG(get_node()->get_logger(), 
                "Received invalid velocity value: %f. Using default: %f", 
                msg->data, default_velocity_);
    current_velocity_magnitude_ = std::abs(default_velocity_);
    current_velocity_ = direction_ * current_velocity_magnitude_;
  }
}

void GripperExampleControllerSimu::positionCommandCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  // Switch to position control mode
  control_mode_ = 1;
  
  // Update target position with the received command
  double requested_position = msg->data;
  
  // Clamp the target position to the allowed range
  if (requested_position > max_opening_) {
    target_position_ = max_opening_;
    RCLCPP_DEBUG(get_node()->get_logger(), 
                "Requested position %f exceeds max opening. Clamping to %f", 
                requested_position, max_opening_);
  } else if (requested_position < min_opening_) {
    target_position_ = min_opening_;
    RCLCPP_DEBUG(get_node()->get_logger(), 
                "Requested position %f below min opening. Clamping to %f", 
                requested_position, min_opening_);
  } else {
    target_position_ = requested_position;
  }
  
  RCLCPP_DEBUG(get_node()->get_logger(), 
              "Switched to position mode. Target position set to: %f", target_position_);
}

void GripperExampleControllerSimu::jointstatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  // Find the indices for finger joint positions
  int finger1_index = -1;
  int finger2_index = -1;
  
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "fr3_finger_joint1") {
      finger1_index = i;
    } else if (msg->name[i] == "fr3_finger_joint2") {
      finger2_index = i;
    }
    
    // Break early if we found both indices
    if (finger1_index != -1 && finger2_index != -1) {
      break;
    }
  }
  
  // Update finger positions if found in the message
  if (finger1_index != -1 && finger2_index != -1) {
    finger1_position_ = msg->position[finger1_index];
    finger2_position_ = msg->position[finger2_index];
    
    // Calculate average position (for more robust position detection)
    double avg_position = (finger1_position_ + finger2_position_) / 2.0;
    
    if (control_mode_ == 0) {  // Velocity control mode
      // Check if we need to change direction
      if (direction_ > 0 && avg_position >= (max_opening_ - position_threshold_)) {
        // Pince is near max opening, reverse to closing direction
        direction_ = -1;
        current_velocity_ = direction_ * current_velocity_magnitude_;
        RCLCPP_DEBUG(get_node()->get_logger(), 
                   "Max opening reached (%f). Reversing to closing direction.", avg_position);
      } 
      else if (direction_ < 0 && avg_position <= (min_opening_ + position_threshold_)) {
        // Pince is near min opening (closed), reverse to opening direction
        direction_ = 1;
        current_velocity_ = direction_ * current_velocity_magnitude_;
        RCLCPP_DEBUG(get_node()->get_logger(), 
                   "Min opening reached (%f). Reversing to opening direction.", avg_position);
      }
    }

    RCLCPP_DEBUG(get_node()->get_logger(), 
                "Gripper positions: f1=%f, f2=%f, avg=%f, mode=%d", 
                finger1_position_, finger2_position_, avg_position, control_mode_);
  }
}

CallbackReturn GripperExampleControllerSimu::on_activate(const rclcpp_lifecycle::State&) {
  if (command_interfaces_.size() >= 2) {
    // Initialize both finger joints with the default velocity
    command_interfaces_[0].set_value(current_velocity_); // finger joint 1
    command_interfaces_[1].set_value(current_velocity_); // finger joint 2
    RCLCPP_INFO(get_node()->get_logger(), "Both gripper command interfaces activated");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Not enough command interfaces found! Expected 2, got %zu", 
                command_interfaces_.size());
    return CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperExampleControllerSimu::on_deactivate(
    const rclcpp_lifecycle::State&) {
  // Set velocity to zero for both fingers when deactivating
  if (command_interfaces_.size() >= 2) {
    command_interfaces_[0].set_value(0.0); // finger joint 1
    command_interfaces_[1].set_value(0.0); // finger joint 2
    RCLCPP_DEBUG(get_node()->get_logger(), "Both gripper velocities set to zero during deactivation");
  }
  
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GripperExampleControllerSimu::update(const rclcpp::Time&,
                                                                  const rclcpp::Duration&) {
  if (command_interfaces_.size() >= 2) {
    if (control_mode_ == 0) {
      // Velocity control mode: Apply the current velocity (with direction) to both fingers
      command_interfaces_[0].set_value(current_velocity_); // finger joint 1
      command_interfaces_[1].set_value(current_velocity_); // finger joint 2
    } else if (control_mode_ == 1) {
      // Position control mode
      
      // Calculate average position
      double avg_position = (finger1_position_ + finger2_position_) / 2.0;
      
      // Calculate position error
      double error = target_position_ - avg_position;
      
      // Calculate velocity based on position error (proportional control)
      double velocity = position_gain_ * error;
      
      // Clamp velocity for safety
      double max_safe_velocity = 0.5;  // Maximum safe velocity
      if (velocity > max_safe_velocity) {
        velocity = max_safe_velocity;
      } else if (velocity < -max_safe_velocity) {
        velocity = -max_safe_velocity;
      }
      
      // Apply calculated velocity to both fingers
      command_interfaces_[0].set_value(velocity); // finger joint 1
      command_interfaces_[1].set_value(velocity); // finger joint 2
      
      // Debug output for position control
      RCLCPP_DEBUG(get_node()->get_logger(), 
                  "Position control: target=%f, current=%f, error=%f, velocity=%f",
                  target_position_, avg_position, error, velocity);
                  
      // If we're close enough to target, reduce the velocity to avoid oscillations
      if (std::abs(error) < position_threshold_) {
        command_interfaces_[0].set_value(0.0);
        command_interfaces_[1].set_value(0.0);
        RCLCPP_DEBUG(get_node()->get_logger(), "Position reached, stopping gripper");
      }
    }
  }
  
  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::GripperExampleControllerSimu,
                       controller_interface::ControllerInterface)