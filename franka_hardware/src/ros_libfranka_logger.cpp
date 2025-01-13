#include "franka_hardware/ros_libfranka_logger.hpp"

namespace franka_hardware {

void RosLibfrankaLogger::logInfo(const std::string& message) {
  RCLCPP_INFO(logger_, "%s", message.c_str());
}

void RosLibfrankaLogger::logWarn(const std::string& message) {
  RCLCPP_WARN(logger_, "%s", message.c_str());
}

void RosLibfrankaLogger::logError(const std::string& message) {
  RCLCPP_ERROR(logger_, "%s", message.c_str());
}

}  // namespace franka_hardware
