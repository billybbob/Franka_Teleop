/*
 * File: TestImpedance.cpp
 * Author: Jerome Perret
 * Copyright: Haption SA, 2021
 * Note: Permission granted to copy, modify, redistribute, provided the above copyright is mentioned
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <array>

/// Include the RaptorAPI
#include "RaptorAPI.hpp"

int main(int argc, char **argv)
{
    HAPTION::ErrorCode error{HAPTION::ErrorCode::E_NOERROR};
	/// Display syntax if wrong number of parameters
	if (argc != 3)
	{
		std::cout << "Syntax: TestCalibration <device_param_file> <protocol_params>" << std::endl;
        std::cout << "Example: TestCalibration virtuose_6d_n187.param channel=SimpleChannelUDP:localip=127.0.0.1:localport=12120:remoteip=192.168.100.53:remoteport=5000" << std::endl;
		return 0;
	}

	/*
	 * Force-feedback device initialization
	 */

	/// Initialize the connection to the device
	HAPTION::RaptorAPI raptorHandle;
    uint8_t major, minor;
    std::string build;
    raptorHandle.GetVersion(major, minor, build);
    std::cout << "RaptorAPI version " << (uint32_t)major << "." << (uint32_t)minor << " build " << build << std::endl;
	raptorHandle.DefineSpecificFields(argv[2]);
    error = raptorHandle.Init(argv[1]);
	if (error != HAPTION::ErrorCode::E_NOERROR)
	{
		std::cerr << "HAPTION::RaptorAPI::Init() failed with error " << (int)error << std::endl;
		std::cerr << "Exiting now!" << std::endl;
        error = HAPTION::ErrorCode::E_NOERROR;
        std::string message;
        raptorHandle.GetLastError(error, message);
        std::cerr << "GetLastError returns " << (int)error << " with message " << message << std::endl;
		return -1;
	}
	/// Display successful connection
	std::string robotName{""};
	(void)raptorHandle.GetFullName(robotName);
	std::cout << "Successfully connected to device " << robotName << std::endl;
	HAPTION::CalibrationStatus status;
	raptorHandle.GetCalibrationStatus(status);
	if (status != HAPTION::CalibrationStatus::C_CALIBRATED)
	{
		std::cerr << "Please calibrate the device first!" << std::endl;
		std::cerr << "Exiting now!" << std::endl;
		raptorHandle.Close();
		return -1;
	}
    /// Authorize force-feedback
    error = raptorHandle.ActivateForceFeedback(true);
    if (error != HAPTION::ErrorCode::E_NOERROR)
    {
        std::cerr << "HAPTION::RaptorAPI::ActivateForceFeedback() failed with error " << (int)error << std::endl;
        std::cerr << "Exiting now!" << std::endl;
        (void)raptorHandle.Close();
        return -1;
    }
    /// Switch to Cartesian mode
    error = raptorHandle.StartCartesianPositionMode();
    if (error != HAPTION::ErrorCode::E_NOERROR)
    {
        std::cerr << "HAPTION::RaptorAPI::StartCartesianPositionMode() failed with error " << (int)error << std::endl;
        std::cerr << "Exiting now!" << std::endl;
        (void)raptorHandle.Close();
        return -1;
    }
    float kT{0.0F};
    float bT{0.0F};
    float kR{0.0F};
    float bR{0.0F};
    error = raptorHandle.GetMaxCartesianGains(kT, bT, kR, bR);
    raptorHandle.ChangeCartesianGains(kT, bT, kR, bR);

    /*
     * Periodic loop
     */
 
    /// This is the time-stamp of the last loop
    auto lastStep = std::chrono::system_clock::now();
    HAPTION::Displacement blockagePose{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    HAPTION::CartesianVector blockageSpeed{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    bool isBlockedPositionSet{false};
    size_t count{ 0U };
    do
    {
        /// Call HAPTION::RaptorAPI::ReadState()
        std::chrono::system_clock::time_point timeStamp;
        error = raptorHandle.ReadState(timeStamp, 0);
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            std::cerr << "HAPTION::RaptorAPI::ReadState() failed with error " << (int)error << std::endl;
            std::cerr << "Exiting now!" << std::endl;
            std::string message;
            error = HAPTION::ErrorCode::E_NOERROR;
            raptorHandle.GetLastError(error, message);
            std::cerr << "GetLastError returns " << (int)error << " with message " << message << std::endl;
            break;
        }
 
        /// Get status of force-feedback
        HAPTION::PowerStatus powerStatus;
        raptorHandle.GetPowerStatus(powerStatus);

        /// Get the current pose and speed of the force-feedback device
        HAPTION::Displacement manipPose;
        HAPTION::CartesianVector manipSpeed;
        (void)raptorHandle.GetCartesianPose(manipPose);
        (void)raptorHandle.GetCartesianSpeed(manipSpeed);

        HAPTION::CartesianVector desCartForce{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
        raptorHandle.GetCartesianForce(desCartForce);
        if (count % 1000 == 0)
        {
            std::cout << "Cartesian Force (N): "
                << desCartForce.t_x << ", "
                << desCartForce.t_y << ", "
                << desCartForce.t_z << ", "
                << desCartForce.r_x << ", "
                << desCartForce.r_y << ", "
                << desCartForce.r_z << std::endl;
    	}

        /// Update the blockage pose if power is off
        if (powerStatus != HAPTION::PowerStatus::P_POWER_DISINHIBITED)
        {
            blockagePose = manipPose;
            blockageSpeed = manipSpeed;
        }
        else
        {
            blockageSpeed.t_x = 0.0;
            blockageSpeed.t_y = 0.0;
            blockageSpeed.t_y = 0.0;
            blockageSpeed.r_x = 0.0;
            blockageSpeed.r_y = 0.0;
            blockageSpeed.r_z = 0.0;
        }

        /// Send the force-feedback device the pose and speed
        raptorHandle.SetCartesianPose(blockagePose);
        raptorHandle.SetCartesianSpeed(blockageSpeed);

        /// Call HAPTION::RaptorAPI::SendSetpoints()
        error = raptorHandle.SendSetpoints();
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            if (error != HAPTION::ErrorCode::E_SAFETYSTOP_POWERBUTTON)
            {
                std::cerr << "HAPTION::RaptorAPI::SendSetpoints() failed with error " << (int)error << std::endl;
                break;
            }
        }
 
        /// Enter busy-wait
        while (true)
        {
            /// Get current time
            auto now = std::chrono::system_clock::now();
            /// Compute duration since last loop in microseconds
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - lastStep).count();
            /// Exit when one millisecond has been reached
            if (duration > 1000)
            {
                lastStep = now;
                break;
            }
        }
        count ++;
 
        /// Push right button to exit
        bool rightButton = false;
        raptorHandle.GetOperatorButton(HAPTION::OperatorButton::B_RIGHT, rightButton);
        if (rightButton)
        {
            break;
        }
    } while (true);
 
    /// Close down
    (void) raptorHandle.Close();
    std::cerr << "Exiting now!" << std::endl;
    return 0;

}
