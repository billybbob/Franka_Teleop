#include <cstdio>
#include <memory>
#include <math.h>
#include "RaptorAPI.hpp"
#include "rclcpp/rclcpp.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_status.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_pose.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_physical_pose.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_speed.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_force.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_pose.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_speed.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_force.hpp"
#include "raptor_api_interfaces/srv/virtuose_calibrate.hpp"
#include "raptor_api_interfaces/srv/virtuose_admittance.hpp"
#include "raptor_api_interfaces/srv/virtuose_impedance.hpp"
#include "raptor_api_interfaces/srv/virtuose_reset.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using namespace HAPTION;
/* Current state of the virtuose node */
#define STATE_UNKNOWN 0
#define STATE_READY 1
#define STATE_ERROR 2
#define STATE_CALIB 3
#define STATE_CART_IMP 4
#define STATE_CART_ADM 5

class RaptorAPIWrapper : public rclcpp::Node
{
public:
    RaptorAPIWrapper()
        : Node("raptor_api_wrapper")
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing publishers");
        outVirtuoseStatusPublisher = this->create_publisher<raptor_api_interfaces::msg::OutVirtuoseStatus>("out_virtuose_status", 1);
        outVirtuosePosePublisher = this->create_publisher<raptor_api_interfaces::msg::OutVirtuosePose>("out_virtuose_pose", 1);
        outVirtuosePhysicalPosePublisher = this->create_publisher<raptor_api_interfaces::msg::OutVirtuosePhysicalPose>("out_virtuose_physical_pose", 1);
        outVirtuoseSpeedPublisher = this->create_publisher<raptor_api_interfaces::msg::OutVirtuoseSpeed>("out_virtuose_speed", 1);
        outVirtuoseForcePublisher = this->create_publisher<raptor_api_interfaces::msg::OutVirtuoseForce>("out_virtuose_force", 1);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing subscriptions");
        inPoseSetPointSubscriber = this->create_subscription<raptor_api_interfaces::msg::InVirtuosePose>("in_virtuose_pose", 1, std::bind(&RaptorAPIWrapper::in_virtuose_poseCB, this, _1));
        inSpeedSetPointSubscriber = this->create_subscription<raptor_api_interfaces::msg::InVirtuoseSpeed>("in_virtuose_speed", 1, std::bind(&RaptorAPIWrapper::in_virtuose_speedCB, this, _1));
        inForceSetPointSubscriber = this->create_subscription<raptor_api_interfaces::msg::InVirtuoseForce>("in_virtuose_force", 1, std::bind(&RaptorAPIWrapper::in_virtuose_forceCB, this, _1));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preparing services");
        calibrateService = this->create_service<raptor_api_interfaces::srv::VirtuoseCalibrate>("virtuose_calibrate", std::bind(&RaptorAPIWrapper::calibrateCB, this, _1, _2));
        admittanceService = this->create_service<raptor_api_interfaces::srv::VirtuoseAdmittance>("virtuose_admittance", std::bind(&RaptorAPIWrapper::admittanceCB, this, _1, _2));
        impedanceService = this->create_service<raptor_api_interfaces::srv::VirtuoseImpedance>("virtuose_impedance", std::bind(&RaptorAPIWrapper::impedanceCB, this, _1, _2));
        // resetService = this->create_service<raptor_api_interfaces::srv::VirtuoseReset>("virtuose_reset", std::bind(&RaptorAPIWrapper::resetCB, this, _1, _2));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raptor api wrapper node ready!");
    }

private:
    // Callback for the topic in_virtuose_pose
    void in_virtuose_poseCB(const raptor_api_interfaces::msg::InVirtuosePose::SharedPtr msg)
    {
        // Check client id
        if (msg->client_id != client_id)
            return;
        // Store last set_pose date
        // set_pose_date = msg->header.stamp.toNSec();
        // Store set_pose
        set_pose[0] = msg->virtuose_pose.translation.x;
        set_pose[1] = msg->virtuose_pose.translation.y;
        set_pose[2] = msg->virtuose_pose.translation.z;
        set_pose[3] = msg->virtuose_pose.rotation.x;
        set_pose[4] = msg->virtuose_pose.rotation.y;
        set_pose[5] = msg->virtuose_pose.rotation.z;
        set_pose[6] = msg->virtuose_pose.rotation.w;
        // Normalize quaternion
        float norm_quat = sqrt(set_pose[3] * set_pose[3] + set_pose[4] * set_pose[4] + set_pose[5] * set_pose[5] + set_pose[6] * set_pose[6]);
        if (norm_quat == 0.0)
        {
            set_pose[6] = 1.0;
        }
        else
        {
            set_pose[3] /= norm_quat;
            set_pose[4] /= norm_quat;
            set_pose[5] /= norm_quat;
            set_pose[6] /= norm_quat;
        }
        isFirstSetpointReceived = true;
        auto _now = now().nanoseconds();
        last_in_date = _now;
        if (ctr % 1000 == 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose set point: %f %f %f %f %f %f %f",set_pose[0],set_pose[1],set_pose[2],set_pose[3],set_pose[4],set_pose[5],set_pose[6]);
        }
        // Watchdog on receiving pose setpoint
        if ((_now - last_loop_date) > 30000000)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timeout on set pose: %d, going to state READY", _now - last_loop_date);
            Reset();
            return;
        }
    }

    // Callback for topic in_virtuose_speed
    void in_virtuose_speedCB(const raptor_api_interfaces::msg::InVirtuoseSpeed::SharedPtr msg)
    {
        // Check client id
        if (msg->client_id != client_id)
            return;
        // Store last set_speed date
        // set_speed_date = msg->header.stamp.toNSec();
        set_speed[0] = msg->virtuose_speed.linear.x;
        set_speed[1] = msg->virtuose_speed.linear.y;
        set_speed[2] = msg->virtuose_speed.linear.z;
        set_speed[3] = msg->virtuose_speed.angular.x;
        set_speed[4] = msg->virtuose_speed.angular.y;
        set_speed[5] = msg->virtuose_speed.angular.z;

        isFirstSetpointReceived = true;
        auto _now = now().nanoseconds();
        last_in_date = _now;
        if (ctr % 1000 == 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speed set point: %f %f %f %f %f %f",set_speed[0],set_speed[1],set_speed[2],set_speed[3],set_speed[4],set_speed[5]);
        }
        // Watchdog on receiving speed setpoint
        if ((_now - last_loop_date) > 30000000)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timeout on set speed: %d, going to state READY", _now - last_loop_date);
            Reset();
            return;
        }
    }

    // Callback for topic in_virtuose_force
    void in_virtuose_forceCB(const raptor_api_interfaces::msg::InVirtuoseForce::SharedPtr msg)
    {
        // Check client id
        if (msg->client_id != client_id || state != STATE_CART_IMP)
            return;
        // Store last set_force date
        // set_force_date = msg->header.stamp.toNSec();
        set_force[0] = msg->virtuose_force.force.x;
        set_force[1] = msg->virtuose_force.force.y;
        set_force[2] = msg->virtuose_force.force.z;
        set_force[3] = msg->virtuose_force.torque.x;
        set_force[4] = msg->virtuose_force.torque.y;
        set_force[5] = msg->virtuose_force.torque.z;
        isFirstSetpointReceived = true;
        auto _now = now().nanoseconds();
        last_in_date = _now;

        if (ctr % 1000 == 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Force set point: %f %f %f %f %f %f", set_force[0], set_force[1], set_force[2], set_force[3], set_force[4], set_force[5]);
        }
        // Watchdog on receiving force setpoint
        if ((_now - last_loop_date) > 30000000)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timeout on set force: %d, going to state READY", _now - last_loop_date);
            Reset();
            return;
        }
    }

    // Callback for service virtuose_calibrate
    bool calibrateCB(const std::shared_ptr<raptor_api_interfaces::srv::VirtuoseCalibrate::Request> request, std::shared_ptr<raptor_api_interfaces::srv::VirtuoseCalibrate::Response> response)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Performing calibration on ip address %s", request->ff_device_ip_address.c_str());
        // Change state to CALIB
        raptor_api_interfaces::msg::OutVirtuoseStatus status;
        status.header.stamp.nanosec = get_clock()->now().nanoseconds();
        status.header.stamp.sec = get_clock()->now().seconds();
        status.header.frame_id = "/Calibration";
        status.state = state = STATE_CALIB;
        outVirtuoseStatusPublisher->publish(status);
        // Initialize the connection to the device
        uint8_t major, minor;
        std::string build;
        raptorHandle.GetVersion(major, minor, build);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "haption_raptor_api package RaptorAPI version: %d.%d, build: %s", major, minor, build.c_str());

        raptorHandle.DefineSpecificFields("channel=" + request->channel + ":localip=" + request->local_ip_address + ":localport=12120:remoteip=" + request->ff_device_ip_address + ":remoteport=5000");

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing RaptorAPI with force feedback param file: %s", request->ff_device_param_file.c_str());
        ErrorCode error = raptorHandle.Init(request->ff_device_param_file);
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            std::cerr << "HAPTION::RaptorAPI::StartLogging() failed with error " << (int)error << std::endl;
            std::cerr << "Exiting now!" << std::endl;
            return -1;
        }
        // Display successful connection
        std::string robotName{""};
        (void)raptorHandle.GetFullName(robotName);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully connected to device %s", robotName.c_str());
        // Get calibration status
        HAPTION::CalibrationStatus calibrationStatus;
        raptorHandle.GetCalibrationStatus(calibrationStatus);
        // Perform calibration if needed
        if (calibrationStatus != HAPTION::CalibrationStatus::C_CALIBRATED)
        {
            // Activate force-feeback (needed only for automatic calibration)
            raptorHandle.ActivateForceFeedback(true);
            do
            {
                HAPTION::CalibrationStatus newStatus;
                // Call "DoCalibration" at regular intervals
                error = raptorHandle.DoCalibration();
                if (error != HAPTION::ErrorCode::E_NOERROR)
                {
                    std::cerr << "HAPTION::RaptorAPI::DoCalibration() failed with error " << (int)error << std::endl;
                    break;
                }
                // A period of 10 milliseconds is adequate
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                // Check new status
                raptorHandle.GetCalibrationStatus(newStatus);
                // Display only if status changed
                if (newStatus != calibrationStatus)
                {
                    switch (newStatus)
                    {
                    case HAPTION::CalibrationStatus::C_WAITINGFORPOWER:
                        std::cout << "Please push the power button!" << std::endl;
                        break;
                    case HAPTION::CalibrationStatus::C_WAITINGFORUSER:
                        std::cout << "Please move the rotation joints to their corresponding calibration positions!" << std::endl;
                        break;
                    case HAPTION::CalibrationStatus::C_MOVING:
                        std::cout << "Warning: the device is moving!" << std::endl;
                        break;
                    case HAPTION::CalibrationStatus::C_CALIBRATED:
                        // std::cout << "Calibration finished. Please remember to depress the power or the calibration button!" << std::endl;
                        break;
                    case HAPTION::CalibrationStatus::C_CALIBRATIONFAILED:
                        std::cout << "Calibration has failed, please restart again" << std::endl;
                        break;
                    case HAPTION::CalibrationStatus::C_IDLE:
                        break;
                    }
                }
                // Update status
                calibrationStatus = newStatus;
                // Exit in case of final success or error
            } while ((calibrationStatus != HAPTION::CalibrationStatus::C_CALIBRATED) && (calibrationStatus != HAPTION::CalibrationStatus::C_CALIBRATIONFAILED));

            if (calibrationStatus == HAPTION::CalibrationStatus::C_CALIBRATED)
            {
                // Change state to READY
                status.header.stamp.nanosec = get_clock()->now().nanoseconds();
                status.header.stamp.sec = get_clock()->now().seconds();
                status.header.frame_id = "/Calibration";
                status.state = state = STATE_READY;
                outVirtuoseStatusPublisher->publish(status);
                // Send response
                response->success = true;
                response->already_calibrated = false;
                response->hardware_down = false;
                response->timeout = false;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration done");
            }
            else
            {
                // Calibration failed
                // Change state to ERROR
                status.header.stamp.nanosec = get_clock()->now().nanoseconds();
                status.header.stamp.sec = get_clock()->now().seconds();
                status.header.frame_id = "/Calibration";
                status.state = state = STATE_ERROR;
                outVirtuoseStatusPublisher->publish(status);
                // Send response
                response->success = false;
                response->already_calibrated = false;
                response->hardware_down = false;
                response->timeout = false;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration failed");
            }
            // Deactivate force-feedback again
            raptorHandle.ActivateForceFeedback(false);
        }
        else
        {
            // Change state to READY
            status.header.stamp.nanosec = get_clock()->now().nanoseconds();
            status.header.stamp.sec = get_clock()->now().seconds();
            status.header.frame_id = "/Calibration";
            status.state = state = STATE_READY;
            outVirtuoseStatusPublisher->publish(status);
            // Send response
            response->success = true;
            response->already_calibrated = true;
            response->hardware_down = false;
            response->timeout = false;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration already done");
        }
        Reset();
        return true;
    }
    // Callback for service virtuose_admittance
    bool admittanceCB(const std::shared_ptr<raptor_api_interfaces::srv::VirtuoseAdmittance::Request> request, std::shared_ptr<raptor_api_interfaces::srv::VirtuoseAdmittance::Response> response)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received admittance request");
        // Check state
        if (state != STATE_READY)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Not in state READY, go to ERROR");
            state = STATE_ERROR;
            response->success = false;
            response->error = true;
            response->client_id = 0;
            return false;
        }
        // Open connection
        start_loop_date = now().nanoseconds();
        ctr = 0;

        HAPTION::Displacement baseAPI;
        baseAPI.t_x = request->base_frame.translation.x;
        baseAPI.t_y = request->base_frame.translation.y;
        baseAPI.t_z = request->base_frame.translation.z;
        baseAPI.q_x = request->base_frame.rotation.x;
        baseAPI.q_y = request->base_frame.rotation.y;
        baseAPI.q_z = request->base_frame.rotation.z;
        baseAPI.q_w = request->base_frame.rotation.w;

        state = STATE_CART_ADM;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Switching to state CART_ADM");

        response->success = true;
        response->error = false;
        response->client_id = generate_client_id();

        uint8_t major, minor;
        std::string build;
        raptorHandle.GetVersion(major, minor, build);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "haption_raptor_api package RaptorAPI version: %d.%d, build: %s", major, minor, build.c_str());

        raptorHandle.DefineSpecificFields("channel=" + request->channel + ":localip=" + request->local_ip_address + ":localport=12120:remoteip=" + request->ff_device_ip_address + ":remoteport=5000");

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing RaptorAPI with force feedback param file: %s", request->ff_device_param_file.c_str());
        ErrorCode error = raptorHandle.Init(request->ff_device_param_file);
        if (error != ErrorCode::E_NOERROR)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: Virtuose Init returned %d", static_cast<int32_t>(error));
            state = STATE_ERROR;
            response->success = false;
            response->error = true;
            response->client_id = 0;
            return false;
        }
        error = raptorHandle.ChangeBase(baseAPI);
        if (error != ErrorCode::E_NOERROR)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error ChangeBase with code %d", static_cast<int32_t>(error));
            response->success = false;
            response->error = true;
            response->client_id = 0;
            state = STATE_ERROR;
            return false;
        }
        // Display successful connection
        std::string robotName{""};
        (void)raptorHandle.GetFullName(robotName);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully connected to device %s", robotName.c_str());
        HAPTION::CalibrationStatus status;
        raptorHandle.GetCalibrationStatus(status);
        if (status != HAPTION::CalibrationStatus::C_CALIBRATED)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Please calibrate the device first!");
            Reset();
            return false;
        }
        error = raptorHandle.ActivateForceFeedback(true);
        if (error != ErrorCode::E_NOERROR)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: Virtuose ActivateForceFeedback returned %d", static_cast<int32_t>(error));
            Reset();
            return false;
        }

        // Switch to Cartesian mode
        error = raptorHandle.StartCartesianPositionMode();
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "HAPTION::RaptorAPI::StartCartesianPositionMode() failed with error %d", (int)error);
            Reset();
            return false;
        }

        /// Set the cartesian Gains
        float32_t kT_master{0.0F};
        float32_t bT_master{0.0F};
        float32_t kR_master{0.0F};
        float32_t bR_master{0.0F};
        error = raptorHandle.GetMaxCartesianGains(kT_master, bT_master, kR_master, bR_master);

        HAPTION::Displacement dispAPI;
        error = raptorHandle.GetTCPOffset(dispAPI);
        if (error != ErrorCode::E_NOERROR)
        {
            std::cerr << "ERROR: GetTCP returned " << static_cast<int32_t>(error) << std::endl;
        }
        float32_t const L{((dispAPI.t_x * dispAPI.t_x) + (dispAPI.t_y * dispAPI.t_y) + (dispAPI.t_z * dispAPI.t_z)) / 1.0F}; // TCP distance
        kT_master = kT_master / (1.0F + ((L * kT_master) / kR_master));                                                      // Change KT of the force feedback robot
        bT_master = bT_master / (1.0F + ((L * bT_master) / bR_master));                                                      // Change BT of the force feedback robot
        kR_master = kR_master / (1.0F + ((L * kT_master) / kR_master));                                                      // Change KR of the force feedback robot
        bR_master = bR_master / (1.0F + ((L * bT_master) / bR_master));                                                      // Change BR of the force feedback robot

        float32_t kT{kT_master};
        float32_t bT{bT_master};
        float32_t kR{kR_master};
        float32_t bR{bR_master};

        if (request->force_factor > 0.0F) // For security
        {
            kT = kT / request->force_factor; // Set KT of target robot
            bT = bT / request->force_factor; // Set bT of target robot
            kR = kR / request->force_factor; // Set KR of target robot
            bR = bR / request->force_factor; // Set bR of target robot

            std::cout << "Virtuose Gains: " << kT << " " << bT << " " << kR << " " << bR << std::endl;
        }

        // Set the master cartesian gains
        error = raptorHandle.ChangeCartesianGains(kT, bT, kR, bR);
        if (error != ErrorCode::E_NOERROR)
        {
            std::cerr << "ERROR: Virtuose ChangeCartesianGains returned " << static_cast<int32_t>(error) << std::endl;
        }

        admittanceThread = this->create_wall_timer(1ms, std::bind(&RaptorAPIWrapper::AdmittanceThread, this));

        return true;
    }

    void AdmittanceThread()
    {

        // This is the time-stamp of the last loop
        state = STATE_CART_ADM;
        auto lastStep = std::chrono::system_clock::now();
        ErrorCode error;
        last_loop_date = now().nanoseconds();

        // Call HAPTION::RaptorAPI::ReadState()
        std::chrono::system_clock::time_point timeStamp;
        error = raptorHandle.ReadState(timeStamp, 0);
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            if (error != HAPTION::ErrorCode::E_SAFETYSTOP_POWERBUTTON)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "HAPTION::RaptorAPI::ReadState() failed with error %d", (int)error);
                Reset();
            }
        }

        // Get status of force-feedback
        HAPTION::PowerStatus powerStatus;
        raptorHandle.GetPowerStatus(powerStatus);

        // Get the current pose and speed of the force-feedback device
        HAPTION::Displacement manipPose;
        (void)raptorHandle.GetCartesianPose(manipPose);
        auto pose = raptor_api_interfaces::msg::OutVirtuosePose();
        pose.header.stamp.nanosec = get_clock()->now().nanoseconds();
        pose.header.stamp.sec = get_clock()->now().seconds();
        pose.virtuose_pose.translation.x = manipPose.t_x;
        pose.virtuose_pose.translation.y = manipPose.t_y;
        pose.virtuose_pose.translation.z = manipPose.t_z;
        pose.virtuose_pose.rotation.x = manipPose.q_x;
        pose.virtuose_pose.rotation.y = manipPose.q_y;
        pose.virtuose_pose.rotation.z = manipPose.q_z;
        pose.virtuose_pose.rotation.w = manipPose.q_w;
        outVirtuosePosePublisher->publish(pose);

        HAPTION::CartesianVector manipSpeed;
        (void)raptorHandle.GetCartesianSpeed(manipSpeed);
        auto vspeed = raptor_api_interfaces::msg::OutVirtuoseSpeed();
        vspeed.header.stamp.nanosec = get_clock()->now().nanoseconds();
        vspeed.header.stamp.sec = get_clock()->now().seconds();
        vspeed.virtuose_speed.linear.x = manipSpeed.t_x;
        vspeed.virtuose_speed.linear.y = manipSpeed.t_y;
        vspeed.virtuose_speed.linear.z = manipSpeed.t_z;
        vspeed.virtuose_speed.angular.x = manipSpeed.r_x;
        vspeed.virtuose_speed.angular.y = manipSpeed.r_y;
        vspeed.virtuose_speed.angular.z = manipSpeed.r_z;
        outVirtuoseSpeedPublisher->publish(vspeed);

        // Status
        auto status = raptor_api_interfaces::msg::OutVirtuoseStatus();
        status.header.stamp.nanosec = get_clock()->now().nanoseconds();
        status.header.stamp.sec = get_clock()->now().seconds();
        status.header.frame_id = "/Admittance";
        status.state = state;
        bool button[2] = {false, false};
        bool emergency_stop;
        raptorHandle.GetOperatorButton(HAPTION::OperatorButton::B_CLUTCH, button[0]);
        raptorHandle.GetOperatorButton(HAPTION::OperatorButton::B_DEADMANSWITCH, button[1]);
        raptorHandle.GetOperatorButton(HAPTION::OperatorButton::B_STOP, emergency_stop);

        status.buttons = ((button[0]) ? 1 : 0) + ((button[1]) ? 2 : 0);

        status.emergency_stop = (emergency_stop);
        outVirtuoseStatusPublisher->publish(status);

        HAPTION::JointVector angles;
        raptorHandle.GetJointAngles(angles);


        // Watchdog on receiving pose setpoint
        if (isFirstSetpointReceived && abs((int64_t)last_in_date - (int64_t)last_loop_date) > 30000000)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timeout on set pose: %d, going to state READY", last_in_date - last_loop_date);
            Reset();
            return;
        }

        // Send pose set point
        HAPTION::Displacement PoseSetPoint{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
        if (isFirstSetpointReceived && (powerStatus == HAPTION::PowerStatus::P_POWER_DISINHIBITED))
        {
            PoseSetPoint.t_x = set_pose[0];
            PoseSetPoint.t_y = set_pose[1];
            PoseSetPoint.t_z = set_pose[2];
            PoseSetPoint.q_x = set_pose[3];
            PoseSetPoint.q_y = set_pose[4];
            PoseSetPoint.q_z = set_pose[5];
            PoseSetPoint.q_w = set_pose[6];
            raptorHandle.SetCartesianPose(PoseSetPoint);
        }
        else{
            raptorHandle.SetCartesianPose(manipPose);
        }
        // Send the force-feedback device its own speed back to avoid drag errors
        HAPTION::CartesianVector SpeedSetPoint{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
        if (isFirstSetpointReceived && (powerStatus == HAPTION::PowerStatus::P_POWER_DISINHIBITED))
        {
            SpeedSetPoint.t_x = set_speed[0];
            SpeedSetPoint.t_y = set_speed[1];
            SpeedSetPoint.t_z = set_speed[2];
            SpeedSetPoint.r_x = set_speed[3];
            SpeedSetPoint.r_y = set_speed[4];
            SpeedSetPoint.r_z = set_speed[5];
            raptorHandle.SetCartesianSpeed(SpeedSetPoint);
        }
        else{
            raptorHandle.SetCartesianSpeed(manipSpeed);
        }

        // Call HAPTION::RaptorAPI::SendSetpoints()
        error = raptorHandle.SendSetpoints();
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "HAPTION::RaptorAPI::SendSetpoints() failed with error %d", (int)error);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose set point: %f %f %f %f %f %f %f",set_pose[0],set_pose[1],set_pose[2],set_pose[3],set_pose[4],set_pose[5],set_pose[6]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speed set point: %f %f %f %f %f %f",set_speed[0],set_speed[1],set_speed[2],set_speed[3],set_speed[4],set_speed[5]);

            Reset();
        }

        ctr++;
        return;
    }

    uint32_t generate_client_id()
    {
        srand(now().nanoseconds());
        client_id = (uint32_t)rand();
        return client_id;
    }

    // Callback for service virtuose_impedance
    bool impedanceCB(const std::shared_ptr<raptor_api_interfaces::srv::VirtuoseImpedance::Request> request, std::shared_ptr<raptor_api_interfaces::srv::VirtuoseImpedance::Response> response)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received impedance request");
        // Check state
        if (state != STATE_READY)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Not in state READY, go to ERROR");
            state = STATE_ERROR;
            response->success = false;
            response->error = true;
            response->client_id = 0;
            return false;
        }
        // Open connection
        start_loop_date = now().nanoseconds();
        ctr = 0;

        HAPTION::Displacement baseAPI;
        baseAPI.t_x = request->base_frame.translation.x;
        baseAPI.t_y = request->base_frame.translation.y;
        baseAPI.t_z = request->base_frame.translation.z;
        baseAPI.q_x = request->base_frame.rotation.x;
        baseAPI.q_y = request->base_frame.rotation.y;
        baseAPI.q_z = request->base_frame.rotation.z;
        baseAPI.q_w = request->base_frame.rotation.w;

        state = STATE_CART_IMP;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Switching to state CART_IMP");

        response->success = true;
        response->error = false;
        response->client_id = generate_client_id();

        uint8_t major, minor;
        std::string build;
        raptorHandle.GetVersion(major, minor, build);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "haption_raptor_api package RaptorAPI version: %d.%d, build: %s", major, minor, build.c_str());

        raptorHandle.DefineSpecificFields("channel=" + request->channel + ":localip=" + request->local_ip_address + ":localport=12120:remoteip=" + request->ff_device_ip_address + ":remoteport=5000");

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing RaptorAPI with force feedback param file: %s", request->ff_device_param_file.c_str());
        ErrorCode error = raptorHandle.Init(request->ff_device_param_file);
        if (error != ErrorCode::E_NOERROR)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: Virtuose Init returned %d", static_cast<int32_t>(error));
            state = STATE_ERROR;
            response->success = false;
            response->error = true;
            response->client_id = 0;
            return false;
        }
        error = raptorHandle.ChangeBase(baseAPI);
        if (error != ErrorCode::E_NOERROR)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error ChangeBase with code %d", static_cast<int32_t>(error));
            response->success = false;
            response->error = true;
            response->client_id = 0;
            state = STATE_ERROR;
            return false;
        }
        // Display successful connection
        std::string robotName{""};
        (void)raptorHandle.GetFullName(robotName);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully connected to device %s", robotName.c_str());
        HAPTION::CalibrationStatus status;
        raptorHandle.GetCalibrationStatus(status);
        if (status != HAPTION::CalibrationStatus::C_CALIBRATED)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Please calibrate the device first!");
            Reset();
            return false;
        }
        error = raptorHandle.ActivateForceFeedback(true);
        if (error != ErrorCode::E_NOERROR)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: Virtuose ActivateForceFeedback returned %d", static_cast<int32_t>(error));
            Reset();
            return false;
        }

        // Switch to Cartesian mode
        error = raptorHandle.StartCartesianPositionMode();
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "HAPTION::RaptorAPI::StartCartesianPositionMode() failed with error %d", (int)error);
            Reset();
            return false;
        }

        /// Set the cartesian Gains
        float32_t kT_master{0.0F};
        float32_t bT_master{0.0F};
        float32_t kR_master{0.0F};
        float32_t bR_master{0.0F};
        error = raptorHandle.GetMaxCartesianGains(kT_master, bT_master, kR_master, bR_master);

        HAPTION::Displacement dispAPI;
        error = raptorHandle.GetTCPOffset(dispAPI);
        if (error != ErrorCode::E_NOERROR)
        {
            std::cerr << "ERROR: GetTCP returned " << static_cast<int32_t>(error) << std::endl;
        }
        float32_t const L{((dispAPI.t_x * dispAPI.t_x) + (dispAPI.t_y * dispAPI.t_y) + (dispAPI.t_z * dispAPI.t_z)) / 1.0F}; // TCP distance
        kT_master = kT_master / (1.0F + ((L * kT_master) / kR_master));                                                      // Change KT of the force feedback robot
        bT_master = bT_master / (1.0F + ((L * bT_master) / bR_master));                                                      // Change BT of the force feedback robot
        kR_master = kR_master / (1.0F + ((L * kT_master) / kR_master));                                                      // Change KR of the force feedback robot
        bR_master = bR_master / (1.0F + ((L * bT_master) / bR_master));                                                      // Change BR of the force feedback robot

        float32_t kT{kT_master};
        float32_t bT{bT_master};
        float32_t kR{kR_master};
        float32_t bR{bR_master};

        if (request->force_factor > 0.0F) // For security
        {
            kT = kT / request->force_factor; // Set KT of target robot
            bT = bT / request->force_factor; // Set bT of target robot
            kR = kR / request->force_factor; // Set KR of target robot
            bR = bR / request->force_factor; // Set bR of target robot

            std::cout << "Virtuose Gains: " << kT << " " << bT << " " << kR << " " << bR << std::endl;
        }

        // Set the master cartesian gains
        error = raptorHandle.ChangeCartesianGains(kT, bT, kR, bR);
        if (error != ErrorCode::E_NOERROR)
        {
            std::cerr << "ERROR: Virtuose ChangeCartesianGains returned " << static_cast<int32_t>(error) << std::endl;
        }

        impedanceThread = this->create_wall_timer(1ms, std::bind(&RaptorAPIWrapper::ImpedanceThread, this));

        return true;
    }
    void ImpedanceThread()
    {

        // This is the time-stamp of the last loop
        state = STATE_CART_IMP;
        auto lastStep = std::chrono::system_clock::now();
        last_loop_date = now().nanoseconds();

        ErrorCode error;
        // Call HAPTION::RaptorAPI::ReadState()
        std::chrono::system_clock::time_point timeStamp;
        error = raptorHandle.ReadState(timeStamp, 0);
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            if (error != HAPTION::ErrorCode::E_SAFETYSTOP_POWERBUTTON)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "HAPTION::RaptorAPI::ReadState() failed with error %d", (int)error);
                Reset();
            }
        }

        // Get status of force-feedback
        HAPTION::PowerStatus powerStatus;
        raptorHandle.GetPowerStatus(powerStatus);

        // Get the current pose and speed of the force-feedback device
        HAPTION::Displacement manipPose;
        (void)raptorHandle.GetCartesianPose(manipPose);
        auto pose = raptor_api_interfaces::msg::OutVirtuosePose();
        pose.header.stamp.nanosec = get_clock()->now().nanoseconds();
        pose.header.stamp.sec = get_clock()->now().seconds();
        pose.virtuose_pose.translation.x = manipPose.t_x;
        pose.virtuose_pose.translation.y = manipPose.t_y;
        pose.virtuose_pose.translation.z = manipPose.t_z;
        pose.virtuose_pose.rotation.x = manipPose.q_x;
        pose.virtuose_pose.rotation.y = manipPose.q_y;
        pose.virtuose_pose.rotation.z = manipPose.q_z;
        pose.virtuose_pose.rotation.w = manipPose.q_w;
        outVirtuosePosePublisher->publish(pose);

        HAPTION::CartesianVector manipSpeed;
        (void)raptorHandle.GetCartesianSpeed(manipSpeed);
        auto vspeed = raptor_api_interfaces::msg::OutVirtuoseSpeed();
        vspeed.header.stamp.nanosec = get_clock()->now().nanoseconds();
        vspeed.header.stamp.sec = get_clock()->now().seconds();
        vspeed.virtuose_speed.linear.x = manipSpeed.t_x;
        vspeed.virtuose_speed.linear.y = manipSpeed.t_y;
        vspeed.virtuose_speed.linear.z = manipSpeed.t_z;
        vspeed.virtuose_speed.angular.x = manipSpeed.r_x;
        vspeed.virtuose_speed.angular.y = manipSpeed.r_y;
        vspeed.virtuose_speed.angular.z = manipSpeed.r_z;
        outVirtuoseSpeedPublisher->publish(vspeed);

        // Status
        auto status = raptor_api_interfaces::msg::OutVirtuoseStatus();
        status.header.stamp.nanosec = get_clock()->now().nanoseconds();
        status.header.stamp.sec = get_clock()->now().seconds();
        status.header.frame_id = "/Impedance";
        status.state = state;
        bool button[2] = {false, false};
        bool emergency_stop;
        raptorHandle.GetOperatorButton(HAPTION::OperatorButton::B_CLUTCH, button[0]);
        raptorHandle.GetOperatorButton(HAPTION::OperatorButton::B_DEADMANSWITCH, button[1]);
        raptorHandle.GetOperatorButton(HAPTION::OperatorButton::B_STOP, emergency_stop);

        status.buttons = ((button[0]) ? 1 : 0) + ((button[1]) ? 2 : 0);

        status.emergency_stop = (emergency_stop);
        outVirtuoseStatusPublisher->publish(status);

        HAPTION::JointVector angles;
        raptorHandle.GetJointAngles(angles);

        // Send the force-feedback device its own pose and speed back to avoid drag errors
        raptorHandle.SetCartesianPose(manipPose);
        raptorHandle.SetCartesianSpeed(manipSpeed);

        // Watchdog on receiving force setpoint
        if (isFirstSetpointReceived && abs((int64_t)last_in_date - (int64_t)last_loop_date) > 30000000)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timeout on set force: %d, going to state READY", last_in_date - last_loop_date);
            Reset();
            return;
        }

        // Add impedance force
        HAPTION::CartesianVector ForceSetPoint{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
        if ((powerStatus == HAPTION::PowerStatus::P_POWER_DISINHIBITED))
        {
            ForceSetPoint.t_x = set_force[0];
            ForceSetPoint.t_y = set_force[1];
            ForceSetPoint.t_z = set_force[2];
            ForceSetPoint.r_x = set_force[3];
            ForceSetPoint.r_y = set_force[4];
            ForceSetPoint.r_z = set_force[5];
        }
        raptorHandle.AddCartesianForceOverlay(ForceSetPoint);

        // Call HAPTION::RaptorAPI::SendSetpoints()
        error = raptorHandle.SendSetpoints();
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "HAPTION::RaptorAPI::SendSetpoints() failed with error %d", (int)error);
            Reset();
        }

        ctr++;
        return;
    }

    // Call back for service virtuose_reset
    bool resetCB(const std::shared_ptr<raptor_api_interfaces::srv::VirtuoseReset::Request> request, std::shared_ptr<raptor_api_interfaces::srv::VirtuoseReset::Response> response)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received Reset request");
        Reset();
        return true;
    }

    void Reset()
    {
        state = STATE_READY;
        client_id = 0;
        isFirstSetpointReceived = false;
        if (impedanceThread != nullptr && !impedanceThread->is_canceled())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopping Impedance thread");
            impedanceThread->cancel();
        }

        if (admittanceThread != nullptr && !admittanceThread->is_canceled())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopping Admittance thread");
            admittanceThread->cancel();
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Closing RaptorAPI Handle");
        raptorHandle.Close();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Switching to state READY");
    }

    /* Publishers */
    rclcpp::Publisher<raptor_api_interfaces::msg::OutVirtuoseStatus>::SharedPtr outVirtuoseStatusPublisher;
    rclcpp::Publisher<raptor_api_interfaces::msg::OutVirtuosePose>::SharedPtr outVirtuosePosePublisher;
    rclcpp::Publisher<raptor_api_interfaces::msg::OutVirtuosePhysicalPose>::SharedPtr outVirtuosePhysicalPosePublisher;
    rclcpp::Publisher<raptor_api_interfaces::msg::OutVirtuoseSpeed>::SharedPtr outVirtuoseSpeedPublisher;
    rclcpp::Publisher<raptor_api_interfaces::msg::OutVirtuoseForce>::SharedPtr outVirtuoseForcePublisher;

    rclcpp::Subscription<raptor_api_interfaces::msg::InVirtuosePose>::SharedPtr inPoseSetPointSubscriber;
    rclcpp::Subscription<raptor_api_interfaces::msg::InVirtuoseSpeed>::SharedPtr inSpeedSetPointSubscriber;
    rclcpp::Subscription<raptor_api_interfaces::msg::InVirtuoseForce>::SharedPtr inForceSetPointSubscriber;

    rclcpp::Service<raptor_api_interfaces::srv::VirtuoseCalibrate>::SharedPtr calibrateService;
    rclcpp::Service<raptor_api_interfaces::srv::VirtuoseImpedance>::SharedPtr impedanceService;
    rclcpp::Service<raptor_api_interfaces::srv::VirtuoseAdmittance>::SharedPtr admittanceService;
    rclcpp::Service<raptor_api_interfaces::srv::VirtuoseReset>::SharedPtr resetService;
    // @brief Virtuose interface
    HAPTION::RaptorAPI raptorHandle;
    /* Setpoints */
    float set_pose[7] = {0, 0, 0, 0, 0, 0, 1};
    uint64_t set_pose_date = 0;
    float set_speed[6];
    uint64_t set_speed_date = 0;
    float set_force[6];
    uint64_t set_force_date = 0;

    /* Dates */
    uint64_t start_loop_date = 0;
    uint64_t last_loop_date = 0;
    uint64_t last_in_date = 0;
    int ctr = 0;
    /* Storage for current client ID */
    uint32_t client_id = 0;
    uint8_t state = STATE_READY;
    rclcpp::TimerBase::SharedPtr impedanceThread;
    rclcpp::TimerBase::SharedPtr admittanceThread;
    bool isFirstSetpointReceived = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RaptorAPIWrapper>());
    rclcpp::shutdown();
    return 0;
}
