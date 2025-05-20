/*
 * File: TestJointImpedance.cpp
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

int ApplicationFunction(std::string const& iParamFile, std::string const& iSpecificFileds, HAPTION::SimpleChannel::SimpleChannelInterface *const iChannel)
{
    /*
	 * Force-feedback device initialization
	 */

    /// Initialize the connection to the device
    HAPTION::RaptorAPI raptorHandle;
    raptorHandle.DefineSpecificFields(iSpecificFileds);
    HAPTION::ErrorCode error = raptorHandle.Init(iParamFile, iChannel);
    if (error != HAPTION::ErrorCode::E_NOERROR)
    {
        std::cerr << "HAPTION::RaptorAPI::Init() failed with error " << (int)error << std::endl;
        std::cerr << "Exiting now!" << std::endl;
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
    /// Don't authorize force-feedback
    error = raptorHandle.ActivateForceFeedback(true);
    if (error != HAPTION::ErrorCode::E_NOERROR)
    {
        std::cerr << "HAPTION::RaptorAPI::ActivateForceFeedback() failed with error " << (int)error << std::endl;
        std::cerr << "Exiting now!" << std::endl;
        (void)raptorHandle.Close();
        return -1;
    }
    /// Switch to Cartesian mode
    error = raptorHandle.StartJointPositionMode();
    if (error != HAPTION::ErrorCode::E_NOERROR)
    {
        std::cerr << "HAPTION::RaptorAPI::StartJointPositionMode() failed with error " << (int)error << std::endl;
        std::cerr << "Exiting now!" << std::endl;
        (void)raptorHandle.Close();
        return -1;
    }
    HAPTION::JointVector jointKGains{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    HAPTION::JointVector jointBGains{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    raptorHandle.ChangeJointGains(jointKGains, jointBGains);
    /*
     * Periodic loop
     */

    /// This is the time-stamp of the last loop
    auto lastStep = std::chrono::system_clock::now();

    /// Counter for display
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
            break;
        }

        /// Get the current pose and speed of the force-feedback device
        HAPTION::JointVector jointPose;
        HAPTION::JointVector jointSpeed;
        HAPTION::Displacement pose;
        (void)raptorHandle.GetJointAngles(jointPose);
        (void)raptorHandle.GetJointSpeeds(jointSpeed);
        (void)raptorHandle.GetCartesianPose(pose);

        if (count % 1000 == 0)
        {
        	std::cout << "Joint angles (deg): "
        		<<  RAD2DEG(jointPose.v[0]) << " "
        		<<  RAD2DEG(jointPose.v[1]) << " "
        		<<  RAD2DEG(jointPose.v[2]) << " "
        		<<  RAD2DEG(jointPose.v[3]) << " "
        		<<  RAD2DEG(jointPose.v[4]) << " "
        		<<  RAD2DEG(jointPose.v[5]) << " "
        		<<  RAD2DEG(jointPose.v[6]) << std::endl;
        	std::cout << "Cartesian pose (m): "
        		<< pose.t_x << " "
        		<< pose.t_y << " "
        		<< pose.t_z << " "
        		<< pose.q_x << " "
        		<< pose.q_y << " "
        		<< pose.q_z << " "
        		<< pose.q_w << std::endl;
        }

        bool signal1;
        bool signal2;       
        raptorHandle.GetOperatorButton(HAPTION::OperatorButton::B_EXTSIGNAL1, signal1);
        raptorHandle.GetOperatorButton(HAPTION::OperatorButton::B_EXTSIGNAL2, signal2);
        if (count % 1000 == 0)
        {
        	std::cout << "External signals: " << signal1 << " " << signal2 << std::endl;
        }
        
        std::array<HAPTION::MotorStatus, HAPTION::MAX_NB_JOINTS> motorsStatus;
        raptorHandle.GetMotorStatus(motorsStatus);
        //std::cout << motorsStatus[0].calibrated << " " << motorsStatus[0].calibrated << " " << motorsStatus[0].calibrated << " " << motorsStatus[0].calibrated << " " 
        //<< motorsStatus[0].calibrated << " " << motorsStatus[0].calibrated << " " << motorsStatus[0].calibrated << std::endl;

        /// Send the force-feedback device its own pose and speed back to avoid drag errors
        raptorHandle.SetJointAngles(jointPose);
        raptorHandle.SetJointSpeeds(jointSpeed);

        /// Call HAPTION::RaptorAPI::SendSetpoints()
        (void)raptorHandle.ChangeBrakeStatus(HAPTION::BrakeStatus::BRAKE_RELEASED);
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
    std::thread device1(ApplicationFunction, argv[1], argv[2], channel1);
    // - Create the second device thread
    std::thread device2(ApplicationFunction, argv[3], argv[4], channel2);
    // - Wait for the first device to terminate
    device1.join();
    // - Wait for the second device to terminate
    device2.join();

    channel1->Close();
    channel2->Close();
    return 0;
}
