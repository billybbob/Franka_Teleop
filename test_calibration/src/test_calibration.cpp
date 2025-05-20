#include <cstdio>
#include <memory>
#include <chrono>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_status.hpp"
#include "raptor_api_interfaces/srv/virtuose_calibrate.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class TestCalibration : public rclcpp::Node
{
public:
  TestCalibration(
      const std::string &name = "test_calibration",
      const std::string &namespace_ = "",
      const rclcpp::NodeOptions &options = (rclcpp::NodeOptions()
                                                .allow_undeclared_parameters(true)
                                                .automatically_declare_parameters_from_overrides(true)))
      : Node(name, namespace_, options)
  {

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing publishers");
    _out_virtuose_status = this->create_subscription<raptor_api_interfaces::msg::OutVirtuoseStatus>("out_virtuose_status", 1, std::bind(&TestCalibration::out_virtuose_statusCB, this, _1));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing services");
    client = this->create_client<raptor_api_interfaces::srv::VirtuoseCalibrate>("virtuose_calibrate");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling calibration service:");
    call_calibration_service();
  }

private:


  // Callback for topic out_virtuose_status
  void out_virtuose_statusCB(const raptor_api_interfaces::msg::OutVirtuoseStatus::SharedPtr msg)
  {
    // Store last status date
    status_date_nanosec = msg->header.stamp.nanosec;
    status_date_sec = msg->header.stamp.sec;
    status_state = msg->state;
    status_button = msg->buttons;
  }

  void call_calibration_service()
  {
    // Request calibration mode
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending calibration request");

    auto imp = std::make_shared<raptor_api_interfaces::srv::VirtuoseCalibrate::Request>();
    this->get_parameter("channel", imp->channel);
    this->get_parameter("ff_device_ip_address", imp->ff_device_ip_address);
    this->get_parameter("ff_device_param_file", imp->ff_device_param_file);
    this->get_parameter("local_ip_address", imp->local_ip_address);

    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(imp);
	bool success{false};
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
	  success = result.get()->success;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration result: %d", success);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service calibration");
      return;
    }

    if (!success)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to calibrate!");
      return;
    }
  }

  // Storage for virtuose pose
  int ctr = 0;

  // Storage for virtuose_node status
  int status_state = 0;
  int status_button;

  uint32_t status_date_nanosec;
  uint32_t status_date_sec;
  uint32_t start;

  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuoseStatus>::SharedPtr _out_virtuose_status;
  rclcpp::Client<raptor_api_interfaces::srv::VirtuoseCalibrate>::SharedPtr client;

};
// Main function
int main(int argc, char **argv)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting test_calibration node");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestCalibration>());
  rclcpp::shutdown();
  return 0;
}