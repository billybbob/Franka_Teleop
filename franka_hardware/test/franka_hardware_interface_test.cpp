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

#include <gmock/gmock.h>
#include <exception>
#include <rclcpp/rclcpp.hpp>

#include <franka_hardware/franka_hardware_interface.hpp>
#include <franka_hardware/model.hpp>
#include <franka_hardware/robot.hpp>

#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "franka/exception.h"

#include <franka_msgs/srv/set_cartesian_stiffness.hpp>
#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>
#include <franka_msgs/srv/set_joint_stiffness.hpp>
#include <franka_msgs/srv/set_load.hpp>
#include <franka_msgs/srv/set_stiffness_frame.hpp>
#include <franka_msgs/srv/set_tcp_frame.hpp>

#include "test_utils.hpp"

#include <fstream>
#include <iostream>

using namespace std::chrono_literals;

class FrankaHardwareInterfaceTest : public ::testing::TestWithParam<std::string> {
 public:
  auto SetUp() -> void override {
    auto urdf_string = readFileToString(TEST_CASE_DIRECTORY + arm_id + ".urdf");
    auto parsed_hardware_infos = hardware_interface::parse_control_resources_from_urdf(urdf_string);
    auto number_of_expected_hardware_components = 1;

    ASSERT_EQ(parsed_hardware_infos.size(), number_of_expected_hardware_components);

    default_hardware_info = parsed_hardware_infos[0];
    default_franka_hardware_interface.on_init(default_hardware_info);
  }

 protected:
  std::string arm_id{"fr3"};
  std::shared_ptr<MockRobot> default_mock_robot = std::make_shared<MockRobot>();
  hardware_interface::HardwareInfo default_hardware_info;
  franka_hardware::FrankaHardwareInterface default_franka_hardware_interface{default_mock_robot,
                                                                             arm_id};

  /* Helper function to get the response of a service */
  template <typename service_client_type,
            typename service_request_type,
            typename service_response_type>
  auto get_param_service_response(
      std::function<void(std::shared_ptr<MockRobot> mock_robot)> mock_function,
      const std::string& service_name,
      service_response_type& response) -> void;
};

template <typename service_client_type,
          typename service_request_type,
          typename service_response_type>
auto FrankaHardwareInterfaceTest::get_param_service_response(
    std::function<void(std::shared_ptr<MockRobot> mock_robot)> mock_function,
    const std::string& service_name,
    service_response_type& response) -> void {
  mock_function(default_mock_robot);

  auto node = rclcpp::Node::make_shared("test_node");

  auto client = node->create_client<service_client_type>(service_name);
  auto request = std::make_shared<service_request_type>();

  RCLCPP_INFO(node->get_logger(), "created request");

  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }

  // Response will be checked on the calling function. Because we can't use ASSERT in a non-void
  // function
  response = *result.get();
}

TEST_F(FrankaHardwareInterfaceTest, givenUnsupportedURDFVersion_thenReturnError) {
  auto urdf_string = readFileToString(TEST_CASE_DIRECTORY + arm_id + "_unsupported_version.urdf");
  auto parsed_hardware_infos = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  auto number_of_expected_hardware_components = 1;

  ASSERT_EQ(parsed_hardware_infos.size(), number_of_expected_hardware_components);

  default_hardware_info = parsed_hardware_infos[0];
  auto return_type = default_franka_hardware_interface.on_init(default_hardware_info);

  ASSERT_EQ(return_type,
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR);
}

TEST_F(FrankaHardwareInterfaceTest, givenFR3ComponentInfo_whenOnInitCalled_expectSuccess) {
  auto urdf_string = readFileToString(TEST_CASE_DIRECTORY + arm_id + ".urdf");
  auto parsed_hardware_infos = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  auto number_of_expected_hardware_components = 1;

  ASSERT_EQ(parsed_hardware_infos.size(), number_of_expected_hardware_components);

  auto return_type = default_franka_hardware_interface.on_init(parsed_hardware_infos[0]);

  ASSERT_EQ(return_type,
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(FrankaHardwareInterfaceTest, givenFR3CommandInterfaces_thenNumberIsSetupCorrectly) {
  const auto command_interfaces = default_franka_hardware_interface.export_command_interfaces();

  const auto number_of_fr3_command_interfaces = 7 + 7 + 7  // for joint command interfaces
                                                + 6    // for cartesian velocity command interfaces
                                                + 2    // for elbow command interfaces
                                                + 16;  // for cartesian pose command interfaces

  ASSERT_EQ(command_interfaces.size(), number_of_fr3_command_interfaces);
}

TEST_F(FrankaHardwareInterfaceTest, givenThatTheRobotInterfacesSet_whenReadCalled_returnOk) {
  franka::RobotState robot_state;
  MockModel mock_model;
  MockModel* model_address = &mock_model;

  EXPECT_CALL(*default_mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*default_mock_robot, getModel()).WillOnce(testing::Return(model_address));

  auto time = rclcpp::Time(0, 0);
  auto duration = rclcpp::Duration(0, 0);
  auto return_type = default_franka_hardware_interface.read(time, duration);
  ASSERT_EQ(return_type, hardware_interface::return_type::OK);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    givenThatTheRobotInterfacesAreSet_whenCallExportState_returnZeroValuesAndCorrectInterfaceNames) {
  franka::RobotState robot_state;
  const size_t state_interface_size = 42;  // position, effort and velocity states for 7*3
                                           // + robot state and model
                                           // + pose(16) + elbow(2)
                                           // + robot_time(1)
  MockModel mock_model;
  MockModel* model_address = &mock_model;

  EXPECT_CALL(*default_mock_robot, getModel()).WillOnce(testing::Return(model_address));
  EXPECT_CALL(*default_mock_robot, readOnce()).WillOnce(testing::Return(robot_state));

  auto time = rclcpp::Time(0);
  auto duration = rclcpp::Duration(0, 0);
  auto return_type = default_franka_hardware_interface.read(time, duration);
  ASSERT_EQ(return_type, hardware_interface::return_type::OK);
  auto states = default_franka_hardware_interface.export_state_interfaces();
  size_t joint_index = 0;

  // Get all the joint states (21 interfaces = 7 joints * 3 interfaces per joint)
  const size_t joint_interfaces = 21;
  for (size_t i = 0; i < joint_interfaces; i++) {
    if (i % 3 == 0) {
      joint_index++;
    }
    const std::string joint_name = k_joint_name + std::to_string(joint_index);
    if (i % 3 == 0) {
      ASSERT_EQ(states[i].get_name(), arm_id + "_" + joint_name + "/" + k_position_controller);
    } else if (i % 3 == 1) {
      ASSERT_EQ(states[i].get_name(), arm_id + "_" + joint_name + "/" + k_velocity_controller);
    } else if (i % 3 == 2) {
      ASSERT_EQ(states[i].get_name(), arm_id + "_" + joint_name + "/" + k_effort_controller);
    }
    ASSERT_EQ(states[i].get_value(), 0.0);
  }

  ASSERT_EQ(states[joint_interfaces].get_name(), arm_id + "/robot_state");
  ASSERT_EQ(states[joint_interfaces + 1].get_name(), arm_id + "/robot_model");
  ASSERT_EQ(states[states.size() - 1].get_name(), arm_id + "/robot_time");

  // Verify total number of interfaces
  ASSERT_EQ(states.size(), state_interface_size);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_that_the_robot_interfaces_are_set_when_call_export_state_interface_robot_model_interface_exists) {
  franka::RobotState robot_state;

  MockModel mock_model;
  MockModel* model_address = &mock_model;
  EXPECT_CALL(*default_mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*default_mock_robot, getModel()).WillOnce(testing::Return(model_address));

  auto time = rclcpp::Time(0);
  auto duration = rclcpp::Duration(0, 0);
  auto return_type = default_franka_hardware_interface.read(time, duration);
  ASSERT_EQ(return_type, hardware_interface::return_type::OK);
  auto states = default_franka_hardware_interface.export_state_interfaces();
  ASSERT_EQ(states[22].get_name(),
            "fr3/robot_model");        // joint states (3*7) + robot state (1)
  EXPECT_NEAR(states[22].get_value(),  // joint states (3*7) + robot state (1)
              *reinterpret_cast<double*>(&model_address),
              k_EPS);  // testing that the casted mock_model ptr
                       // is correctly pushed to state interface
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_that_the_robot_interfaces_are_set_when_call_export_state_interface_robot_state_interface_exists) {
  franka::RobotState robot_state;
  franka::RobotState* robot_state_address = &robot_state;

  MockModel mock_model;
  MockModel* model_address = &mock_model;
  EXPECT_CALL(*default_mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*default_mock_robot, getModel()).WillOnce(testing::Return(model_address));

  auto time = rclcpp::Time(0);
  auto duration = rclcpp::Duration(0, 0);
  auto return_type = default_franka_hardware_interface.read(time, duration);
  ASSERT_EQ(return_type, hardware_interface::return_type::OK);
  auto states = default_franka_hardware_interface.export_state_interfaces();
  ASSERT_EQ(states[21].get_name(),
            "fr3/robot_state");  // joint states (3*7) , then comes robot state
  EXPECT_NEAR(states[21].get_value(), *reinterpret_cast<double*>(&robot_state_address),
              k_EPS);  // testing that the casted robot state ptr
                       // is correctly pushed to state interface
}

TEST_P(FrankaHardwareInterfaceTest,
       when_prepare_command_mode_interface_for_stop_effort_interfaces_expect_ok) {
  std::string command_interface = GetParam();

  std::vector<std::string> stop_interface;

  for (size_t i = 0; i < default_hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    stop_interface.push_back(joint_name + "/" + command_interface);
  }
  std::vector<std::string> start_interface = {};
  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
}

TEST_P(
    FrankaHardwareInterfaceTest,
    when_prepare_command_mode_interface_is_called_with_invalid_start_interface_number_expect_throw) {
  std::string command_interface = GetParam();

  std::vector<std::string> stop_interface;

  for (size_t i = 0; i < default_hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    stop_interface.push_back(joint_name + "/" + command_interface);
  }
  std::vector<std::string> start_interface = {"fr3_joint1/effort"};
  EXPECT_THROW(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                             stop_interface),
               std::invalid_argument);
}

TEST_P(FrankaHardwareInterfaceTest,
       when_prepare_command_mode_interface_for_start_effort_interfaces_expect_ok) {
  std::string command_interface = GetParam();

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < default_hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + command_interface);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
}

TEST_P(
    FrankaHardwareInterfaceTest,
    when_prepare_command_mode_interface_is_called_with_invalid_stop_interface_number_expect_throw) {
  std::string command_interface = GetParam();

  std::vector<std::string> start_interface, stop_interface;

  for (size_t i = 0; i < default_hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    stop_interface.push_back(joint_name + "/" + command_interface);
  }

  start_interface = {"fr3_joint1/effort"};

  EXPECT_THROW(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                             stop_interface),
               std::invalid_argument);
}

TEST_P(FrankaHardwareInterfaceTest, whenWriteCalled_expectOk) {
  std::string command_interface = GetParam();

  EXPECT_CALL(*default_mock_robot, writeOnce(std::vector<double>{0, 0, 0, 0, 0, 0, 0}));

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < default_hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + command_interface);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  if (command_interface == k_position_controller) {
    ASSERT_EQ(default_franka_hardware_interface.read(time, duration),
              hardware_interface::return_type::OK);
  }
  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::OK);
}

TEST_F(FrankaHardwareInterfaceTest, whenWriteCalledWithInifiteCommand_expectError) {
  franka::RobotState robot_state;
  robot_state.q = std::array<double, 7>{std::numeric_limits<double>::infinity()};

  EXPECT_CALL(*default_mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*default_mock_robot, writeOnce(std::vector<double>{0, 0, 0, 0, 0, 0, 0})).Times(0);

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < default_hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + k_position_controller);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(default_franka_hardware_interface.read(time, duration),
            hardware_interface::return_type::OK);
  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::ERROR);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    givenPositionJointCommandInterfaceInitialized_ifWriteCalledWithoutRead_expectWriteOnceNotToBeCalled) {
  EXPECT_CALL(*default_mock_robot, writeOnce(std::vector<double>{0, 0, 0, 0, 0, 0, 0})).Times(0);

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < default_hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + k_position_controller);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::OK);
}

TEST_F(FrankaHardwareInterfaceTest, when_on_activate_called_expect_success) {
  franka::RobotState robot_state;

  MockModel mock_model;
  MockModel* model_address = &mock_model;

  EXPECT_CALL(*default_mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*default_mock_robot, getModel()).WillOnce(testing::Return(model_address));

  ASSERT_EQ(default_franka_hardware_interface.on_activate(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(FrankaHardwareInterfaceTest, when_on_deactivate_called_expect_success) {
  EXPECT_CALL(*default_mock_robot, stopRobot());

  ASSERT_EQ(default_franka_hardware_interface.on_deactivate(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_P(FrankaHardwareInterfaceTest,
       given_start_effort_interface_prepared_when_perform_command_mode_switch_called_expect_ok) {
  std::string command_interface = GetParam();

  if (command_interface == k_effort_controller) {
    EXPECT_CALL(*default_mock_robot, initializeTorqueInterface());
  } else if (command_interface == k_velocity_controller) {
    EXPECT_CALL(*default_mock_robot, initializeJointVelocityInterface());
  } else if (command_interface == k_position_controller) {
    EXPECT_CALL(*default_mock_robot, initializeJointPositionInterface());
  }

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < default_hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + command_interface);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
}

TEST_P(FrankaHardwareInterfaceTest,
       given_that_effort_control_started_perform_command_mode_switch_stop_expect_ok) {
  std::string command_interface = GetParam();

  EXPECT_CALL(*default_mock_robot, stopRobot()).Times(2);

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < default_hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + command_interface);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  for (size_t i = 0; i < default_hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    stop_interface.push_back(joint_name + "/" + command_interface);
  }

  start_interface.clear();

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_joint_stiffness_service_called_expect_robot_set_joint_stiffness_to_be_called) {
  auto expect_call_set_joint_stiffness = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setJointStiffness(testing::_)).Times(1);
  };
  franka_msgs::srv::SetJointStiffness::Response response;
  get_param_service_response<franka_msgs::srv::SetJointStiffness,
                             franka_msgs::srv::SetJointStiffness::Request,
                             franka_msgs::srv::SetJointStiffness::Response>(
      expect_call_set_joint_stiffness, "service_server/set_joint_stiffness", response);

  ASSERT_TRUE(response.success);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_joint_cartesian_service_called_expect_robot_set_joint_cartesian_to_be_called) {
  auto expect_call_set_cartesian_stiffness = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setCartesianStiffness(testing::_)).Times(1);
  };
  franka_msgs::srv::SetCartesianStiffness::Response response;
  get_param_service_response<franka_msgs::srv::SetCartesianStiffness,
                             franka_msgs::srv::SetCartesianStiffness::Request,
                             franka_msgs::srv::SetCartesianStiffness::Response>(
      expect_call_set_cartesian_stiffness, "service_server/set_cartesian_stiffness", response);

  ASSERT_TRUE(response.success);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_load_service_called_expect_robot_set_load_to_be_called) {
  auto expect_call_set_load = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setLoad(testing::_)).Times(1);
  };
  franka_msgs::srv::SetLoad::Response response;
  get_param_service_response<franka_msgs::srv::SetLoad, franka_msgs::srv::SetLoad::Request,
                             franka_msgs::srv::SetLoad::Response>(
      expect_call_set_load, "service_server/set_load", response);

  ASSERT_TRUE(response.success);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_tcp_frame_service_called_expect_robot_set_tcp_frame_to_be_called) {
  auto expect_call_set_tcp_frame = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setTCPFrame(testing::_)).Times(1);
  };
  franka_msgs::srv::SetTCPFrame::Response response;
  get_param_service_response<franka_msgs::srv::SetTCPFrame, franka_msgs::srv::SetTCPFrame::Request,
                             franka_msgs::srv::SetTCPFrame::Response>(
      expect_call_set_tcp_frame, "service_server/set_tcp_frame", response);

  ASSERT_TRUE(response.success);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_stiffness_frame_service_called_expect_robot_set_stiffness_frame_to_be_called) {
  auto expect_call_set_stiffness_frame = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setStiffnessFrame(testing::_)).Times(1);
  };
  franka_msgs::srv::SetStiffnessFrame::Response response;
  get_param_service_response<franka_msgs::srv::SetStiffnessFrame,
                             franka_msgs::srv::SetStiffnessFrame::Request,
                             franka_msgs::srv::SetStiffnessFrame::Response>(
      expect_call_set_stiffness_frame, "service_server/set_stiffness_frame", response);

  ASSERT_TRUE(response.success);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_force_torque_collision_behavior_service_called_expect_same_function_in_robot_class_to_be_called) {
  auto expect_call_set_force_torque_collision_behavior =
      [&](std::shared_ptr<MockRobot> mock_robot) {
        EXPECT_CALL(*mock_robot, setForceTorqueCollisionBehavior(testing::_)).Times(1);
      };
  franka_msgs::srv::SetForceTorqueCollisionBehavior::Response response;
  get_param_service_response<franka_msgs::srv::SetForceTorqueCollisionBehavior,
                             franka_msgs::srv::SetForceTorqueCollisionBehavior::Request,
                             franka_msgs::srv::SetForceTorqueCollisionBehavior::Response>(
      expect_call_set_force_torque_collision_behavior,
      "service_server/set_force_torque_collision_behavior", response);

  ASSERT_TRUE(response.success);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_full_collision_behavior_service_called_expect_same_function_in_robot_class_to_be_called) {
  auto expect_call_set_full_collision_behavior = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setFullCollisionBehavior(testing::_)).Times(1);
  };
  franka_msgs::srv::SetFullCollisionBehavior::Response response;
  get_param_service_response<franka_msgs::srv::SetFullCollisionBehavior,
                             franka_msgs::srv::SetFullCollisionBehavior::Request,
                             franka_msgs::srv::SetFullCollisionBehavior::Response>(
      expect_call_set_full_collision_behavior, "service_server/set_full_collision_behavior",
      response);

  ASSERT_TRUE(response.success);
}

TEST_F(FrankaHardwareInterfaceTest, set_joint_stiffness_throws_error) {
  auto set_joint_stiffness_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setJointStiffness(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };
  franka_msgs::srv::SetJointStiffness::Response response;
  get_param_service_response<franka_msgs::srv::SetJointStiffness,
                             franka_msgs::srv::SetJointStiffness::Request,
                             franka_msgs::srv::SetJointStiffness::Response>(
      set_joint_stiffness_mock_throw, "service_server/set_joint_stiffness", response);

  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

TEST_F(FrankaHardwareInterfaceTest, set_cartesian_stiffness_throws_error) {
  auto set_cartesian_stiffness_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setCartesianStiffness(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };
  franka_msgs::srv::SetCartesianStiffness::Response response;
  get_param_service_response<franka_msgs::srv::SetCartesianStiffness,
                             franka_msgs::srv::SetCartesianStiffness::Request,
                             franka_msgs::srv::SetCartesianStiffness::Response>(
      set_cartesian_stiffness_mock_throw, "service_server/set_cartesian_stiffness", response);
  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

TEST_F(FrankaHardwareInterfaceTest, set_load_throws_error) {
  auto set_load_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setLoad(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };
  franka_msgs::srv::SetLoad::Response response;
  get_param_service_response<franka_msgs::srv::SetLoad, franka_msgs::srv::SetLoad::Request,
                             franka_msgs::srv::SetLoad::Response>(
      set_load_mock_throw, "service_server/set_load", response);

  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

TEST_F(FrankaHardwareInterfaceTest, set_EE_frame_throws_error) {
  auto set_tcp_frame_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setTCPFrame(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };
  franka_msgs::srv::SetTCPFrame::Response response;
  get_param_service_response<franka_msgs::srv::SetTCPFrame, franka_msgs::srv::SetTCPFrame::Request,
                             franka_msgs::srv::SetTCPFrame::Response>(
      set_tcp_frame_mock_throw, "service_server/set_tcp_frame", response);
  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

TEST_F(FrankaHardwareInterfaceTest, set_K_frame_throws_error) {
  auto set_stiffness_frame_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setStiffnessFrame(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };
  franka_msgs::srv::SetStiffnessFrame::Response response;
  get_param_service_response<franka_msgs::srv::SetStiffnessFrame,
                             franka_msgs::srv::SetStiffnessFrame::Request,
                             franka_msgs::srv::SetStiffnessFrame::Response>(
      set_stiffness_frame_mock_throw, "service_server/set_stiffness_frame", response);
  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

TEST_F(FrankaHardwareInterfaceTest, set_force_torque_collision_behavior_throws_error) {
  auto set_force_torque_collision_behavior_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setForceTorqueCollisionBehavior(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };

  franka_msgs::srv::SetForceTorqueCollisionBehavior::Response response;
  get_param_service_response<franka_msgs::srv::SetForceTorqueCollisionBehavior,
                             franka_msgs::srv::SetForceTorqueCollisionBehavior::Request,
                             franka_msgs::srv::SetForceTorqueCollisionBehavior::Response>(
      set_force_torque_collision_behavior_mock_throw,
      "service_server/set_force_torque_collision_behavior", response);
  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

TEST_F(FrankaHardwareInterfaceTest, set_full_collision_behavior_throws_error) {
  auto set_full_collision_behavior_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setFullCollisionBehavior(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };
  franka_msgs::srv::SetFullCollisionBehavior::Response response;
  get_param_service_response<franka_msgs::srv::SetFullCollisionBehavior,
                             franka_msgs::srv::SetFullCollisionBehavior::Request,
                             franka_msgs::srv::SetFullCollisionBehavior::Response>(
      set_full_collision_behavior_mock_throw, "service_server/set_full_collision_behavior",
      response);
  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

int main(int argc, char** argv) {
  rclcpp::init(0, nullptr);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

INSTANTIATE_TEST_SUITE_P(FrankaHardwareTests,
                         FrankaHardwareInterfaceTest,
                         ::testing::Values(k_velocity_controller,
                                           k_effort_controller,
                                           k_position_controller));
