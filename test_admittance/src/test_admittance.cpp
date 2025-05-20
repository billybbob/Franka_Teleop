#include <cstdio>
#include <memory>
#include <chrono>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_status.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_pose.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_physical_pose.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_speed.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_force.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_pose.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_speed.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_force.hpp"
#include "raptor_api_interfaces/srv/virtuose_admittance.hpp"
#include "raptor_api_interfaces/srv/virtuose_reset.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class TestAdmittance : public rclcpp::Node
{
public:
  TestAdmittance(
      const std::string &name = "test_admittance",
      const std::string &namespace_ = "",
      const rclcpp::NodeOptions &options = (rclcpp::NodeOptions()
                                                .allow_undeclared_parameters(true)
                                                .automatically_declare_parameters_from_overrides(true)))
      : Node(name, namespace_, options)
  {

    this->get_parameter("horizontal_table_z", horizontal_table_z);
    this->get_parameter("max_force", max_force);
    this->get_parameter("penalty_stiffness", penalty_stiffness);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing subscriptions");
    _out_virtuose_status = this->create_subscription<raptor_api_interfaces::msg::OutVirtuoseStatus>("out_virtuose_status", 1, std::bind(&TestAdmittance::out_virtuose_statusCB, this, _1));
    _out_virtuose_pose = this->create_subscription<raptor_api_interfaces::msg::OutVirtuosePose>("out_virtuose_pose", 1, std::bind(&TestAdmittance::out_virtuose_poseCB, this, _1));
    _out_virtuose_speed = this->create_subscription<raptor_api_interfaces::msg::OutVirtuoseSpeed>("out_virtuose_speed", 1, std::bind(&TestAdmittance::out_virtuose_speedCB, this, _1));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing publisher");
    _in_virtuose_pose = this->create_publisher<raptor_api_interfaces::msg::InVirtuosePose>("in_virtuose_pose", 1);
    _in_virtuose_speed = this->create_publisher<raptor_api_interfaces::msg::InVirtuoseSpeed>("in_virtuose_speed", 1);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing services");
    client = this->create_client<raptor_api_interfaces::srv::VirtuoseAdmittance>("virtuose_admittance");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling admittance service:");
    call_admittance_service();
  }

private:
  // Callback for topic out_virtuose_pose
  void out_virtuose_poseCB(const raptor_api_interfaces::msg::OutVirtuosePose::SharedPtr msg)
  {
    // Store last pose date
    pose_date_nanosec = msg->header.stamp.nanosec;
    pose_date_sec = msg->header.stamp.sec;
    cur_pose[0] = msg->virtuose_pose.translation.x;
    cur_pose[1] = msg->virtuose_pose.translation.y;
    cur_pose[2] = msg->virtuose_pose.translation.z;
    cur_pose[3] = msg->virtuose_pose.rotation.x;
    cur_pose[4] = msg->virtuose_pose.rotation.y;
    cur_pose[5] = msg->virtuose_pose.rotation.z;
    cur_pose[6] = msg->virtuose_pose.rotation.w;
    if (ctr % 1000 == 0)
    {
      // printf("Virtuose pose: %f %f %f %f %f %f %f\n", cur_pose[0], cur_pose[1], cur_pose[2], cur_pose[3], cur_pose[4], cur_pose[5], cur_pose[6]);
    }
  }

  void out_virtuose_speedCB(const raptor_api_interfaces::msg::OutVirtuoseSpeed::SharedPtr msg)
  {
    // Store last pose date
    pose_date_nanosec = msg->header.stamp.nanosec;
    pose_date_sec = msg->header.stamp.sec;
    cur_speed[0] = msg->virtuose_speed.linear.x;
    cur_speed[1] = msg->virtuose_speed.linear.y;
    cur_speed[2] = msg->virtuose_speed.linear.z;
    cur_speed[3] = msg->virtuose_speed.angular.x;
    cur_speed[4] = msg->virtuose_speed.angular.y;
    cur_speed[5] = msg->virtuose_speed.angular.z;
    if (ctr % 1000 == 0)
    {
      // printf("Virtuose pose: %f %f %f %f %f %f %f\n", cur_pose[0], cur_pose[1], cur_pose[2], cur_pose[3], cur_pose[4], cur_pose[5], cur_pose[6]);
    }
  }

  // Callback for topic out_virtuose_status
  void out_virtuose_statusCB(const raptor_api_interfaces::msg::OutVirtuoseStatus::SharedPtr msg)
  {
    // Store last status date
    status_date_nanosec = msg->header.stamp.nanosec;
    status_date_sec = msg->header.stamp.sec;
    status_state = msg->state;
    status_button = msg->buttons;
  }

  void call_admittance_service()
  {
    // Request admittance mode
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending admittance request");

    auto adm = std::make_shared<raptor_api_interfaces::srv::VirtuoseAdmittance::Request>();
    this->get_parameter("channel", adm->channel);
    this->get_parameter("ff_device_ip_address", adm->ff_device_ip_address);
    this->get_parameter("ff_device_param_file", adm->ff_device_param_file);
    this->get_parameter("local_ip_address", adm->local_ip_address);
    this->get_parameter("speed_factor", adm->speed_factor);
    this->get_parameter("force_factor", adm->force_factor);
    adm->base_frame.translation.x = 0.0;
    adm->base_frame.translation.y = 0.0;
    adm->base_frame.translation.z = 0.0;
    adm->base_frame.rotation.x = 0.0;
    adm->base_frame.rotation.y = 0.0;
    adm->base_frame.rotation.z = 0.0;
    adm->base_frame.rotation.w = 1.0;

    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(adm);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      // Store client ID given by virtuose_node
      client_id = result.get()->client_id;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Our client ID is: %ld", client_id);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service admittance");
      return;
    }

    if (client_id == 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service admittance, client_id is zero!");
      return;
    }

    ctr = 0;
    // Perform admittance loop at 1000 Hz
    admittanceThread = this->create_wall_timer(1ms, std::bind(&TestAdmittance::AdmittanceThread, this));
  }

  void AdmittanceThread()
  {

    // Copy the current pose to set point
    raptor_api_interfaces::msg::InVirtuosePose pose;
    pose.header.stamp.nanosec = get_clock()->now().nanoseconds();
    pose.header.stamp.sec = get_clock()->now().seconds();
    pose.client_id = client_id;
    pose.virtuose_pose.translation.x = cur_pose[0];
    pose.virtuose_pose.translation.y = cur_pose[1];
    pose.virtuose_pose.translation.z = cur_pose[2];
    pose.virtuose_pose.rotation.x = cur_pose[3];
    pose.virtuose_pose.rotation.y = cur_pose[4];
    pose.virtuose_pose.rotation.z = cur_pose[5];
    pose.virtuose_pose.rotation.w = cur_pose[6];
    // Simulate virtual wall at z=-10 cm
    if (cur_pose[2] < horizontal_table_z)
    {
      pose.virtuose_pose.translation.z = horizontal_table_z;
    }
    _in_virtuose_pose->publish(pose);

    // Copy the current speed to set point
    raptor_api_interfaces::msg::InVirtuoseSpeed speed;
    speed.header.stamp.nanosec = get_clock()->now().nanoseconds();
    speed.header.stamp.sec = get_clock()->now().seconds();
    speed.client_id = client_id;
    speed.virtuose_speed.linear.x = cur_speed[0];
    speed.virtuose_speed.linear.y = cur_speed[1];
    speed.virtuose_speed.linear.z = cur_speed[2];
    speed.virtuose_speed.angular.x = cur_speed[3];
    speed.virtuose_speed.angular.y = cur_speed[4];
    speed.virtuose_speed.angular.z = cur_speed[5];
    _in_virtuose_speed->publish(speed);
    ctr++;
    uint64_t dt = get_clock()->now().nanoseconds() - (long long unsigned int)status_date_sec * 1000000000U + (long long unsigned int)status_date_nanosec;
    // Print status every second
    if (ctr % 1000 == 0)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Status: %llu %d %d %f %f %f", dt, status_state, status_button,
             cur_pose[0], cur_pose[1], cur_pose[2]);
    }
  }

  // Storage for virtuose pose
  float cur_pose[7];
  float cur_speed[6];
  uint32_t pose_date_nanosec;
  uint32_t pose_date_sec;
  int ctr = 0;

  // Storage for virtuose_node status
  int status_state = 0;
  int status_button;
  uint32_t client_id = 0;

  uint32_t status_date_nanosec;
  uint32_t status_date_sec;
  uint32_t start;

  float penalty_stiffness, horizontal_table_z, max_force;

  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuoseStatus>::SharedPtr _out_virtuose_status;
  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuosePose>::SharedPtr _out_virtuose_pose;
  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuoseSpeed>::SharedPtr _out_virtuose_speed;
  rclcpp::Subscription<raptor_api_interfaces::msg::OutVirtuoseForce>::SharedPtr _out_virtuose_force;
  rclcpp::Publisher<raptor_api_interfaces::msg::InVirtuosePose>::SharedPtr _in_virtuose_pose;
  rclcpp::Publisher<raptor_api_interfaces::msg::InVirtuoseSpeed>::SharedPtr _in_virtuose_speed;
  rclcpp::Client<raptor_api_interfaces::srv::VirtuoseAdmittance>::SharedPtr client;

  rclcpp::TimerBase::SharedPtr admittanceThread;
};
// Main function
int main(int argc, char **argv)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting test_admittance node");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestAdmittance>());
  rclcpp::shutdown();
  return 0;
}