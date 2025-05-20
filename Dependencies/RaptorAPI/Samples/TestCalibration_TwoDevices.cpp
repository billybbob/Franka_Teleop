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
#include "SimpleChannelInterface.hpp"

#if defined(LINUX)
#include <time.h>
#include <string.h>
#include <dlfcn.h>
#else
#include <Windows.h>
#endif

float RAD2DEG(float &a)
{
    return static_cast<float>((a / M_PI) * 180.0);
}
float DEG2RAD(float &a)
{
    return static_cast<float>((a * M_PI) / 180.0);
}

int CalibrateDevice(std::string const& iParamFile, std::string const& iSpecificFileds, HAPTION::SimpleChannel::SimpleChannelInterface *const iChannel)
{
    std::cout << "ThreadFunction: " << iParamFile << " " << iSpecificFileds << std::endl;
    /*
	 * Force-feedback device initialization
	 */
	// Initialize the connection to the device
	HAPTION::RaptorAPI raptorHandle;
    uint8_t major, minor;
    std::string build;
    raptorHandle.GetVersion(major, minor, build);
	raptorHandle.DefineSpecificFields(iSpecificFileds);
	HAPTION::ErrorCode error = raptorHandle.Init(iParamFile, iChannel);
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
	HAPTION::CalibrationStatus status;
	raptorHandle.GetCalibrationStatus(status);
	if (status != HAPTION::CalibrationStatus::C_CALIBRATED)
	{
		std::cerr << "Please calibrate the device first!" << std::endl;
		//std::cerr << "Exiting now!" << std::endl;
		//raptorHandle.Close();
		//return -1;
	}
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
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
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
	}
	else
	{
		std::cout << "Device is already calibrated!" << std::endl;
	}

    // Close down
    (void)raptorHandle.Close();
    std::cerr << "Exiting now!" << std::endl;
    return 0;
}

int main(int argc, char **argv)
{
	// Display syntax if wrong number of parameters
	if (argc != 5)
	{
		std::cout << "Syntax: TestTwoDevices <device1_param_file> <protocol1_params> <device2_param_file> <protocol2_params>" << std::endl;
        std::cout << "Example: TestCalibration virtuose_6d_n187.param remoteip=192.168.100.53:remoteport=5000 virtuose_6d_n188.param remoteip=192.168.100.54:remoteport=5000" << std::endl;
		return 0;
	}
	std::cout << "Args: " << argv[1] << " " << argv[2] << std::endl;

    HAPTION::SimpleChannel::SimpleChannelInterface *channel1{ nullptr };
#ifdef WIN32
    HINSTANCE dllHandle1;
    std::string const dll_path{"SimpleChannelCIFX.dll"};
    dllHandle1 = LoadLibrary(dll_path.c_str());
    if (dllHandle1 == nullptr)
    {
        std::cout << "Unable to load DLL " + dll_path << std::endl;
    }

    // Get the function from the DLL
    void *factory_ptr1{nullptr};
    factory_ptr1 = GetProcAddress(dllHandle1, "ChannelFactory");
    if (factory_ptr1 == nullptr)
    {
        std::cout << "Unable to load ChannelFactory from DLL" << std::endl;
        int32_t res{0};
        res = FreeLibrary(dllHandle1);
        if (res == 0)
        {
            std::cout << "Freelibrary failed" << std::endl;
        }
    }
#else
    // Load the DLL
    void* dllHandle1;
    std::string const dll_path{"libSimpleChannelCIFX.so"};
    dllHandle1 = dlopen(dll_path.c_str(), RTLD_NOW | RTLD_GLOBAL | RTLD_NODELETE);
    if (dllHandle1 == nullptr)
    {
        std::cout << "Unable to load DLL " + dll_path << std::endl;
    }
    // Get the function from the DLL
    void *factory_ptr1{nullptr};
    factory_ptr1 = dlsym(dllHandle1, "ChannelFactory");
    if (factory_ptr1 == nullptr)
    {
        std::cout << "Unable to load ChannelFactory from DLL" << std::endl;
        int32_t res{0};
        res = dlclose(dllHandle1);
        if (res == 0)
        {
            std::cout << "Freelibrary failed" << std::endl;
        }
    }
#endif
    // Cast the factory function
    HAPTION::SimpleChannel::SimpleChannelInterface *(*factory_func1)();
    factory_func1 = reinterpret_cast<HAPTION::SimpleChannel::SimpleChannelInterface *(*)()>(factory_ptr1);
    // Create the procotol interface
    channel1 = factory_func1();

    HAPTION::SimpleChannel::SimpleChannelInterface *channel2{ nullptr };
#ifdef WIN32
    HINSTANCE dllHandle2;
    dllHandle2 = LoadLibrary(dll_path.c_str());
    if (dllHandle2 == nullptr)
    {
        std::cout << "Unable to load DLL " + dll_path << std::endl;
    }
    // Get the function from the DLL
    void *factory_ptr2{nullptr};
    factory_ptr2 = GetProcAddress(dllHandle2, "ChannelFactory");
    if (factory_ptr2 == nullptr)
    {
        std::cout << "Unable to load ChannelFactory from DLL" << std::endl;
        int32_t res{0};
        res = FreeLibrary(dllHandle2);
        if (res == 0)
        {
            std::cout << "Freelibrary failed" << std::endl;
        }
    }
#else
    // Load the DLL
    void* dllHandle2;
    dllHandle2 = dlopen(dll_path.c_str(), RTLD_NOW | RTLD_GLOBAL | RTLD_NODELETE);
    if (dllHandle2 == nullptr)
    {
        std::cout << "Unable to load DLL " + dll_path << std::endl;
    }
    // Get the function from the DLL
    void *factory_ptr2{nullptr};
    factory_ptr2 = dlsym(dllHandle2, "ChannelFactory");
    if (factory_ptr2 == nullptr)
    {
        std::cout << "Unable to load ChannelFactory from DLL" << std::endl;
        int32_t res{0};
        res = dlclose(dllHandle2);
        if (res == 0)
        {
            std::cout << "Freelibrary failed" << std::endl;
        }
    }
#endif
    // Cast the factory function
    HAPTION::SimpleChannel::SimpleChannelInterface *(*factory_func2)();
    factory_func2 = reinterpret_cast<HAPTION::SimpleChannel::SimpleChannelInterface *(*)()>(factory_ptr2);
    // Create the procotol interface
    channel2 = factory_func2();

    channel1->Open(argv[2]);
    channel2->Open(argv[4]);

    // - Create the first device thread
    std::thread device1(CalibrateDevice, argv[1], argv[2], channel1);
    // - Create the second device thread
    std::thread device2(CalibrateDevice, argv[3], argv[4], channel2);
    // - Wait for the first device to terminate
    device1.join();
    // - Wait for the second device to terminate
    device2.join();

    channel1->Close();
    channel2->Close();
    return 0;
}
