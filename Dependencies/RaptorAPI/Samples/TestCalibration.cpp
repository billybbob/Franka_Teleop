/*
 * File: TestCalibration.cpp
 * Author: Jerome Perret
 * Copyright: Haption SA, 2021
 * Note: Permission granted to copy, modify, redistribute, provided the above copyright is mentioned
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <array>

// Include the RaptorAPI
#include "RaptorAPI.hpp"

// Unique function of the program
int main(int argc, char **argv)
{
	// Display syntax if wrong number of parameters
	if (argc != 3)
	{
		std::cout << "Syntax: TestCalibration <device_param_file> <protocol_params>" << std::endl;
        std::cout << "Example: TestCalibration virtuose_6d_n187.param channel=SimpleChannelUDP:localip=127.0.0.1:localport=12120:remoteip=192.168.100.53:remoteport=5000" << std::endl;
		return 0;
	}

	/*
	 * Force-feedback device initialization
	 */

	// Initialize the connection to the device
	HAPTION::RaptorAPI raptorHandle;
    uint8_t major, minor;
    std::string build;
    raptorHandle.GetVersion(major, minor, build);
    std::cout << "RaptorAPI version " << (uint32_t)major << "." << (uint32_t)minor << " build " << build << std::endl;
	HAPTION::ErrorCode error = raptorHandle.DefineSpecificFields(argv[2]);
	if (error != HAPTION::ErrorCode::E_NOERROR)
	{
		std::string message;
		raptorHandle.GetLastError(error, message);
		std::cerr << "HAPTION::RaptorAPI::DefineSpecificFields() failed with error " << (int)error << std::endl;
		std::cerr << message << std::endl;
		std::cerr << "Exiting now!" << std::endl;
		return -1;
	}
	error = raptorHandle.Init(argv[1]);
	if (error != HAPTION::ErrorCode::E_NOERROR)
	{
		std::string message;
		raptorHandle.GetLastError(error, message);
		std::cerr << "HAPTION::RaptorAPI::Init() failed with error " << (int)error << std::endl;
		std::cerr << message << std::endl;
		std::cerr << "Exiting now!" << std::endl;
		return -1;
	}
	// Display successful connection
	std::string robotName{""};
	(void)raptorHandle.GetFullName(robotName);
	std::cout << "Successfully connected to device " << robotName << std::endl;
	// Get calibration status
	HAPTION::CalibrationStatus calibrationStatus;
	raptorHandle.GetCalibrationStatus(calibrationStatus);
	// Perform calibration if needed
	if (calibrationStatus == HAPTION::CalibrationStatus::C_CALIBRATED)
	{
		std::cout << "Device already calibrated" << std::endl;
	}
	else
	{
		// Activate force-feeback (needed only for automatic calibration)
		raptorHandle.ActivateForceFeedback(true);
		auto top = std::chrono::system_clock::now();
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

			// Busy-wait for 10 milliseconds
			while(true)
			{
				auto now = std::chrono::system_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - top);
				if (duration.count() > 2)
				{
					top = now;
					break;
				}
			}
			
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
					std::cout << "Calibration finished. Please remember to depress the power or the calibration button!" << std::endl;
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
		// Deactivate force-feedback again
		raptorHandle.ActivateForceFeedback(false);
		if (error == HAPTION::ErrorCode::E_NOERROR)
		{
			std::cout << "Calibration succeeded" << std::endl;
		}
		else
		{
			std::cout << "Calibration failed" << std::endl;
		}
	}
	// Close down
	(void)raptorHandle.Close();
	std::cerr << "Exiting now!" << std::endl;
	return 0;
}
