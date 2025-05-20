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

float RAD2DEG(float &a)
{
    return float((a / M_PI) * 180.0f);
}
float DEG2RAD(float &a)
{
    return float((a * M_PI) / 180.0f);
}

int main(int argc, char **argv)
{
	float weight = 0.0F;
	/// Display syntax if wrong number of parameters
	if ((argc != 3) && (argc != 4))
	{
		std::cout << "Syntax: TestImpedance <device_param_file> <protocol_params> <weight>" << std::endl;
        std::cout << "Example: TestImpedance virtuose_6d_n187.param channel=SimpleChannelUDP:localip=127.0.0.1:localport=12120:remoteip=192.168.100.53:remoteport=5000" << std::endl;
		return 0;
	}
	if (argc == 4)
	{
		weight = (float)atof(argv[3]);
		if ((weight < 0.0F) || (weight > 30.0F))
		{
			weight = 0.0F;
		}
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
    HAPTION::ErrorCode error = raptorHandle.Init(argv[1]);
	if (error != HAPTION::ErrorCode::E_NOERROR)
	{
        /// Get DLL version information
        std::string dllVersion{""};
        if (raptorHandle.GetParameterValue("RaptorAPI", "DeviceDLLVersion", dllVersion) == HAPTION::ErrorCode::E_NOERROR)
        {
            std::cout << "Device DLL version is: " << dllVersion << std::endl;
        }
		std::cerr << "HAPTION::RaptorAPI::Init() failed with error " << (int)error << std::endl;
		std::cerr << "Exiting now!" << std::endl;
		return -1;
	}
	/// Display successful connection
	std::string robotName{""};
	(void)raptorHandle.GetFullName(robotName);
	std::cout << "Successfully connected to device " << robotName << std::endl;
    /// Get DLL version information
    std::string dllVersion{""};
    if (raptorHandle.GetParameterValue("RaptorAPI", "DeviceDLLVersion", dllVersion) == HAPTION::ErrorCode::E_NOERROR)
    {
        std::cout << "Device DLL version is: " << dllVersion << std::endl;
    }
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
    raptorHandle.ChangeCartesianGains(0.0F, 0.0F, 0.0F, 0.0F);
    
    /*
     * Periodic loop
     */
 
    /// This is the time-stamp of the last loop
    auto lastStep = std::chrono::system_clock::now();
 
    do
    {
        /// Call HAPTION::RaptorAPI::ReadState()
        std::chrono::system_clock::time_point timeStamp;
        error = raptorHandle.ReadState(timeStamp, 0);
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            if (error != HAPTION::ErrorCode::E_SAFETYSTOP_POWERBUTTON)
            {
                std::cerr << "HAPTION::RaptorAPI::ReadState() failed with error " << (int)error << std::endl;
                std::cerr << "Exiting now!" << std::endl;
                break;
            }
        }
 
        /// Get status of force-feedback
        HAPTION::PowerStatus powerStatus;
        raptorHandle.GetPowerStatus(powerStatus);

        /// Get the current pose and speed of the force-feedback device
        HAPTION::Displacement manipPose;
        HAPTION::CartesianVector manipSpeed;
        (void)raptorHandle.GetCartesianPose(manipPose);
        (void)raptorHandle.GetCartesianSpeed(manipSpeed);

        HAPTION::JointVector angles;
        raptorHandle.GetJointAngles(angles);

        /// Simulate a virtual table at Z = -10 cm
        HAPTION::CartesianVector repulsionForce{ 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        if ((powerStatus == HAPTION::PowerStatus::P_POWER_DISINHIBITED) && (manipPose.t_z < -0.1F))
        {
            /// Apply pure stiffness of 1000 N/m
            repulsionForce.t_z = - (manipPose.t_z + 0.1F) * 1000.0F;
        }
 
        /// Send the force-feedback device its own pose and speed back to avoid drag errors
        raptorHandle.SetCartesianPose(manipPose);
        raptorHandle.SetCartesianSpeed(manipSpeed);

        /// Add impedance force
        raptorHandle.AddCartesianForceOverlay(repulsionForce);
 
        /// Call HAPTION::RaptorAPI::SendSetpoints()
        error = raptorHandle.SendSetpoints();
        if (error != HAPTION::ErrorCode::E_NOERROR)
        {
            std::cerr << "HAPTION::RaptorAPI::SendSetpoints() failed with error " << (int)error << std::endl;
            break;
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
