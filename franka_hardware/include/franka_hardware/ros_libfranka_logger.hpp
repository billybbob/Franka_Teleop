#pragma once

#include <rclcpp/rclcpp.hpp>

#include <franka/logging/logging_sink_interface.hpp>

namespace franka_hardware {

/**
 * Implements the logger interface of libfranka to use the ROS logger.
 */
class RosLibfrankaLogger : public franka::LoggingSinkInterface {
 public:
  explicit RosLibfrankaLogger(const rclcpp::Logger& logger) : logger_(logger) {}
  ~RosLibfrankaLogger() override = default;

  // Inherited via LoggingSinkInterface
  auto getName() const -> std::string override { return "RosLibfrankaLogger"; }
  auto logInfo(const std::string& message) -> void override;
  auto logWarn(const std::string& message) -> void override;
  auto logError(const std::string& message) -> void override;

 private:
  rclcpp::Logger logger_;
};

}  // namespace franka_hardware
