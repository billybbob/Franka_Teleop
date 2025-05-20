/*!
 * \file RaptorAPI.hpp
 * This file contains the definition of the RaptorAPI class.
 * \copyright Haption SA
 */

#ifndef H_RAPTOR_API
#define H_RAPTOR_API

#include <string>
#include <chrono>

#include "RobotTypes.hpp"
#include "RaptorAPI_Export.h"
#include "SimpleChannelInterface.hpp"

namespace HAPTION
{
	/// Upfront declaration of the internal members
	class RaptorAPIInternals;

	/// @brief Robotic Application Programming Interface for haptic devices produced by Haption
	class RaptorAPI_EXPORT RaptorAPI final
	{
	public:
		/// @brief Explicit declaration of the constructor (mandatory for AUTOSAR "rule of six" A12-0-1)
		RaptorAPI() noexcept = default;

		/// @brief Explicit declaration of the copy constructor (mandatory for AUTOSAR "rule of six" A12-0-1)
		RaptorAPI(RaptorAPI const&) noexcept = default;

		/// @brief Explicit declaration of the copy operator (mandatory for AUTOSAR "rule of six" A12-0-1)
		RaptorAPI& operator=(RaptorAPI const&) & noexcept = default;

		/// @brief Explicit declaration of the copy constructor (mandatory for AUTOSAR "rule of six" A12-0-1)
		RaptorAPI(RaptorAPI&&) noexcept = default;

		/// @brief Explicit declaration of the copy operator (mandatory for AUTOSAR "rule of six" A12-0-1)
		RaptorAPI& operator=(RaptorAPI&&) & noexcept = default;

		/// @brief Explicit declaration of the destructor.
		/// The only action of the destructor is to call the method RaptorAPI::Close(). <br/>
		/// It's advisable to call RaptorAPI::Close() explicitly instead, so as to catch any potential error!
		~RaptorAPI() noexcept { (void)Close(); }

		/// @brief Get the version of the RaptorAPI
		/// @param[out] oMajor Major version
		/// @param[out] oMinor Minor version
		/// @param[out] oBuild Build idenfication, typically formed of the subversion revision and the compile date
		void GetVersion(uint8_t &oMajor, uint8_t &oMinor, std::string &oBuild) const noexcept;

		/// @brief Get the version of the firmware of the device
		/// @param oMajor Major version
		/// @param oMinor Minor version
		/// @param oBuild Build idenfication, typically the compile date in the form YYYYMMDD (in decimal)
		/// @param oSignature Cryptographic signature (SHA265) of the firmware code
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTAVAILABLE if not implemented <br/>
		/// E_NOTCONNECTED if not connected to a device
		ErrorCode GetFirmwareVersion(uint8_t &oMajor, uint8_t &oMinor, uint32_t &oBuild, std::array<uint8_t, 32> &oSignature) noexcept;

		/// @brief Define field values needed by the specific device.
		/// Example for a UDP-connected Virtuose or Desktop: "ip=192.168.100.53:port=5000" <br/>
		/// Example for an EtherCAT-connected Virtuose with CIFx card on Microsoft Windows: "slot=1:offset=200" <br/>
		/// Example for an EtherCAT-connected Virtuose with CIFx card on Linux: "card=0:slot=1:offset=200" <br/>
		/// Example for a UDP-connected Achille: "channel=SimpleChannelUDP:localip=192.168.100.150:localport=12120:remoteip=192.168.100.185:remoteport=5000" <br/>
		/// Example for an EtherCAT-connected Achille with CIFx card on Windows: "channel=SimpleChannelCIFX:slot=1:offset=200" <br/>
		/// Example for an EtherCAT-connected Achille with CIFx card on Linux: "channel=SimpleChannelCIFX:card=0:slot=1:offset=200" <br/>
		/// @note This function needs to be called BEFORE Init()
		/// @param[in] iSpecificFields List of associations "field=value" separated by semicolons
		/// @return E_NOERROR in case of success <br/>
		/// E_MEMORYALLOCATION in case of memory allocation problem
		ErrorCode DefineSpecificFields(std::string const &iSpecificFields) noexcept;

		/// @brief Get the last error report.
		/// @param[out] oErrorCode Last reported error code
		/// @param[out] oErrorMessage Short description of the last reported error
		static void GetLastError(ErrorCode &oErrorCode, std::string &oErrorMessage) noexcept;

		/// @brief Initialize the class object and try to connect to the device with the given configuration file.
		/// @ref Init
		/// @param iName Full path of param file, or param file name if local, or name of device-specific DLL without prefix and suffix (input)
		/// @param iChannel Pointer to the channel to be used for communication with the device (must be open, default nullptr)
		/// @return E_MEMORYALLOCATION in case of insufficient memory in the heap <br/>
		/// E_FILENOTFOUND if the configuration file cannot be accessed <br/>
		/// E_WRONGPARAMETER in case of error in the configuration file, or parameter missing <br/>
		/// E_NOTAVAILABLE in case of: <ul>
		/// <li> Calling Init() while already connected to a device </li>
		/// <li> Short read of the configuration file </li>
		/// <li> Error in loading the device-specific library </li>
		/// <li> Error in looking for the factory function in the device-specific library </li>
		/// <li> Error in loading the channel-specific library </li>
		/// <li> Error in looking for the factory function in the channel-specific library </li>
		/// <li> Error in opening the communication channel </li>
		/// </ul><br/>
		/// E_NOTCONNECTED in case of device not responding <br/>
		/// E_WRONGSERIALNUMBER in case of wrong device serial number <br/>
		/// E_MEMORYALLOCATION in case of memory allocation problem <br/>
		/// E_NOERROR in case of success
		ErrorCode Init(std::string const &iName, SimpleChannel::SimpleChannelInterface *const iChannel = nullptr) noexcept;

		/// @brief Get device full name.
		/// This is available after a successful call to Init().
		/// @param[out] oName Full name of the device
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before or connection has failed
		ErrorCode GetFullName(std::string &oName) noexcept;

		/// @brief Close the connection and terminate.
		/// This is available after a successful call to Init()
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode Close() noexcept;

		/// @brief Get the calibration status of the device.
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oIsCalibrated Calibration status
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode GetCalibrationStatus(CalibrationStatus &oIsCalibrated) noexcept;

		/// @brief Get the error status of the device.
		/// A call to ReadState() will refresh the error status.
		/// @return E_NOTCONNECTED if not connected to a force-feedback device <br/>
		/// Else the error status of the device
		ErrorCode GetErrorStatus() noexcept;

		/// @brief Perform the device calibration.
		/// This is available after a successfull call to Init(). <br/>
		/// This needs to be called at regular intervals for the calibration to proceed!
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device, or if connection fails <br/>
		/// E_WRONGPARAMETER if configuration is wrong
		ErrorCode DoCalibration() noexcept;

		/// @brief Set the base transformation.
		/// This is available only after a successful call to Init() and before any "Start" function has been called.
		/// @param[in] iDisp Transformation from viewpoint frame to base frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if the parameter is not a valid displacement <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before
		ErrorCode ChangeBase(Displacement const &iDisp) noexcept;

		/// @brief Set the base transformation.
		/// This is available only after a successful call to Init() and before any "Start" function has been called. <br/>
		/// This function is provided for convenience, but the RaptorAPI is using Displacement internally.
		/// @param[in] iTransformation Transformation matrix from viewpoint frame to base frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if the parameter is not a valid transformation <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before
		ErrorCode ChangeBase(Transformation const &iTransformation) noexcept;

		/// @brief Set the viewpoint transformation.
		/// This is available at any time between Init() and Close(). <br/>
		/// This call modifies the clutch offset.
		/// @param[in] iDisp Transformation from world frame to viewpoint frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if the parameter is not a valid displacement <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before
		ErrorCode ChangeViewpoint(Displacement const &iDisp) noexcept;

		/// @brief Set the viewpoint transformation.
		/// This is available at any time between Init() and Close(). <br/>
		/// This call modifies the clutch offset.
		/// @param[in] iTransformation Transformation matrix from world frame to viewpoint frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if the parameter is not a valid transformation <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before
		ErrorCode ChangeViewpoint(Transformation const &iTransformation) noexcept;

		/// @brief Change continuously the viewpoint transformation (this might generate forces!).
		/// This is available at any time between Init() and Close(). <br/>
		/// This call modifies the clutch offset.
		/// @param[in] iDisp Transformation from world frame to viewpoint frame
		/// @param[in] iSpeed Motion from world frame to viewpoint frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if iDisp is not a valid displacement or iSpeed a valid vector <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before
		ErrorCode MoveViewpoint(Displacement const &iDisp, CartesianVector const &iSpeed) noexcept;

		/// @brief Change continuously the viewpoint frame (this might generate forces!).
		/// This is available at any time between Init() and Close(). <br/>
		/// This call modifies the clutch offset.
		/// @param[in] iTransformation Transformation from world frame to viewpoint frame
		/// @param[in] iSpeed Motion from world frame to viewpoint frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if iTransformation is not a valid transformation or iSpeed a valid vector <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before
		ErrorCode MoveViewpoint(Transformation const &iTransformation, CartesianVector const &iSpeed) noexcept;

		/// @brief Change movement scaling.
		/// This is available at any time between Init() and Close(). <br/>
		/// This call modifies the clutch offset.
		/// @param[in] iScaleTrans Scaling of translations
		/// @param[in] iScaleRot Scaling of rotations
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if iScaleTrans or iScaleRot is NaN, infinite or negative or null <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before
		ErrorCode ChangeMovementScale(float32_t const iScaleTrans, float32_t const iScaleRot) noexcept;

		/// @brief Get the current (relative) Cartesian pose.
		/// This is available after a successful call to Init(). <br/>
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oPose Current Cartesian pose in the world frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before
		ErrorCode GetCartesianPose(Displacement &oPose) noexcept;

		/// @brief Get the current (relative) Cartesian pose.
		/// This is available after a successful call to Init(). <br/>
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oPose Current Cartesian pose in the world frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before
		ErrorCode GetCartesianPose(Transformation &oPose) noexcept;

		/// @brief Get the current (absolute) Cartesian pose
		/// This function returns the current absolute cartesian pose in the
		/// base frame without including the clutch offset, movement scale,
		/// base orientation, and observation frame. <br/>
		/// This is available after a successful call to Init()
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oPose Current Cartesian pose in the base frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before
		ErrorCode GetRawCartesianPose(Displacement &oPose) noexcept;

		/// @brief Get the current (absolute) Cartesian pose
		/// This function returns the current absolute cartesian pose in the
		/// base frame without including the clutch offset, movement scale,
		/// base orientation, and observation frame. <br/>
		/// This is available after a successful call to Init()
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oPose Current Cartesian pose in the base frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before
		ErrorCode GetRawCartesianPose(Transformation &oPose) noexcept;

		/// @brief Get the current (relative) Cartesian speed.
		/// This is available after a successful call to Init(). <br/>
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oSpeed Current Cartesian speed in the world frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before
		ErrorCode GetCartesianSpeed(CartesianVector &oSpeed) noexcept;

		/// @brief Get the current joint angles.
		/// This is available after a successful call to Init(). <br/>
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oAngles Current joint angles in rad
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before <br/>
		/// E_WRONGPARAMETER if incompatible number of joints
		ErrorCode GetJointAngles(JointVector &oAngles) noexcept;

		/// @brief Get the current joint speeds.
		/// This is available after a successful call to Init(). <br/>
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oSpeeds Current joint rotation speeds in rad/s
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before <br/>
		/// E_WRONGPARAMETER if incompatible number of joints
		ErrorCode GetJointSpeeds(JointVector &oSpeeds) noexcept;

		/// @brief Get the current measured joint torques.
		/// This function calculates the joints torques from raw motors currents. <br/>
		/// This is available after a successful call to Init(). <br/>
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oTorques Current joint torques in N/m
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before or if not supported by the device <br/>
		/// E_WRONGPARAMETER if incompatible number of joints
		ErrorCode GetJointTorques(JointVector &oTorques) noexcept;

		/// @brief Get the measured motors currents.
		/// This is available after a successful call to Init(). <br/>
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oCurrents Current motors current in A
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before or if not supported by the device <br/>
		/// E_WRONGPARAMETER if incompatible number of joints
		/// @note Not available on all devices (Achille-specific)
		ErrorCode GetMotorCurrents(JointVector &oCurrents) noexcept;

		/// @brief Get the detailed status of the motors.
		/// This is available after a successful call to Init(). <br/>
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oStatus Current motors status
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before or if not supported by the device <br/>
		/// E_WRONGPARAMETER if incompatible number of joints
		/// @note Not available on all devices (Achille-specific)
		ErrorCode GetMotorStatus(std::array<MotorStatus, MAX_NB_JOINTS> &oStatus) noexcept;

		/// @brief Get the detailed status of the primary position encoders.
		/// This is available after a successful call to Init(). <br/>
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oSensorsStatus Current sensor status
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before or if not supported by the device <br/>
		/// E_WRONGPARAMETER if incompatible number of joints
		/// @note Not available on all devices (Achille-specific)
		ErrorCode GetPrimarySensorsStatus(std::array<SensorStatus, MAX_NB_JOINTS> &oSensorsStatus) noexcept;

		/// @brief Get the detailed status of the redundant position encoders.
		/// This is available after a successful call to Init(). <br/>
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oSensorsStatus Current sensor status
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before or if not supported by the device <br/>
		/// E_WRONGPARAMETER if incompatible number of joints
		/// @note Not available on all devices (Achille-specific)
		ErrorCode GetRedundantSensorsStatus(std::array<SensorStatus, MAX_NB_JOINTS> &oSensorsStatus) noexcept;

		/// @brief Get the current raw encoder values
		/// This is available after a successful call to Init(). <br/>
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param oRawValue Raw encoder measurement
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before or if not supported by the device <br/>
		/// @note Not available on all devices (Achille-specific)
		ErrorCode GetRawEncoderValue(JointVector &oRawValue) noexcept;

		/// @brief Change the Cartesian gains.
		/// This is available after a successful call to StartCartesianPositionMode().
		/// @param[in] iKT Proportional gain on translation, in N/m
		/// @param[in] iBT Derivate gain on translation, in N/(m/s)
		/// @param[in] iKR Proportional gain on rotation, in Nm/rad
		/// @param[in] iBR Derivate gain on rotation, in Nm/(rad/s)
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before or if not supported by the device <br/>
		/// E_WRONGPARAMETER if one of the values is NaN, infinite, negative, or larger than the maximum allowed value
		ErrorCode ChangeCartesianGains(float32_t iKT, float32_t iBT, float32_t iKR, float32_t iBR) noexcept;

		/// @brief Get the maximum value of the cartesian gains.
		/// This is available after a successful call to StartCartesianPositionMode().
		/// @param[out] oMaxKT Maximum proportional gain on translation, in N/m
		/// @param[out] oMaxBT Maximum derivate gain on translation, in N/(m/s)
		/// @param[out] oMaxKR Maximum proportional gain on rotation, in Nm/rad
		/// @param[out] oMaxBR Maximum derivate gain on rotation, in Nm/(rad/s)
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode GetMaxCartesianGains(float32_t &oMaxKT, float32_t &oMaxBT, float32_t &oMaxKR, float32_t &oMaxBR) noexcept;

		/// @brief Start Cartesian position control mode.
		/// This is available after a successful call to Init().
		/// @note This method resets the Cartesian position gains to zero. <br/>
		/// A call to RaptorAPI::ChangeCartesianGains is needed for setting non-zero gains afterwards.
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode StartCartesianPositionMode() noexcept;

		/// @brief Change the joint-space gains.
		/// This is available after a successful call to StartJointPositionMode().
		/// @param[in] iKs Proportional gains for all joints, in Nm/rad
		/// @param[in] iBs Derivate gains for all joints, in Nm/(rad/s)
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before or if not supported by the device <br/>
		/// E_WRONGPARAMETER if one of the values is NaN, infinite, negative, or larger than the maximum allowed value
		ErrorCode ChangeJointGains(JointVector const &iKs, JointVector const &iBs) noexcept;

		/// @brief Get the maximum value of the joint gains.
		/// This is available after a successful call to StartJointPositionMode().
		/// @param[out] oMaxKs Maximum proportional gains for all joints, in Nm/rad
		/// @param[out] oMaxBs Maximum derivate gains for all joints, in Nm/(rad/s)
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if wrong number of joints
		ErrorCode GetMaxJointGains(JointVector &oMaxKs, JointVector &oMaxBs) noexcept;

		/// @brief Start joint position control mode.
		/// This is available after a successful call to Init().
		/// @note This method resets the joint position gains to zero. <br/>
		/// A call to RaptorAPI::ChangeJointGains is needed for setting non-zero gains afterwards.
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode StartJointPositionMode() noexcept;

		/// @brief Activate or deactivate force-feedback.
		/// This is available after a successful call to Init().
		/// @param[in] iActivate True to activate force-feedback, false to deactivate
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode ActivateForceFeedback(bool const iActivate) noexcept;

		/// @brief Set the value of a floating-point extension register.
		/// This is available after a successful call to Init().
		/// @param[in] iRegID ID of the register
		/// @param[in] iValue Value as a float
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if register ID unknown <br/>
		/// E_NOTAVAILABLE if not supported by the device
		/// @note Not available on all devices
		ErrorCode SetExtRegister(size_t const iRegID, float32_t const iValue) noexcept;

		/// @brief Set the value of a boolean extension register.
		/// This is available after a successful call to Init().
		/// @param[in] iRegID ID of the register
		/// @param[in] iValue Value as a boolean
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if register ID unknown <br/>
		/// E_NOTAVAILABLE if not supported by the device
		/// @note Not available on all devices
		ErrorCode SetExtRegister(size_t const iRegID, bool const iValue) noexcept;

		/// @brief Set the value of a string extension register.
		/// This is available after a successful call to Init().
		/// @param[in] iRegID ID of the register
		/// @param[in] iValue Value as a string
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if register ID unknown <br/>
		/// E_NOTAVAILABLE if not supported by the device
		/// @note Not available on all devices
		ErrorCode SetExtRegister(size_t const iRegID, std::string const& iValue) noexcept;

		/// @brief Get the value of a floating-point extension register.
		/// This is available after a successful call to Init().
		/// @param[in] iRegID ID of the register
		/// @param[out] oValue Value as a float
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if register ID unknown <br/>
		/// E_NOTAVAILABLE if not supported by the device
		/// @note Not available on all devices
		ErrorCode GetExtRegister(size_t const iRegID, float32_t &oValue) noexcept;

		/// @brief Get the value of a boolean extension register.
		/// This is available after a successful call to Init().
		/// @param[in] iRegID ID of the register
		/// @param[out] oValue Value as a boolean
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if register ID unknown <br/>
		/// E_NOTAVAILABLE if not supported by the device
		/// @note Not available on all devices
		ErrorCode GetExtRegister(size_t const iRegID, bool &oValue) noexcept;

		/// @brief Get the value of a string extension register.
		/// This is available after a successful call to Init().
		/// @param[in] iRegID ID of the register
		/// @param[out] oValue Value as a string
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if register ID unknown <br/>
		/// E_NOTAVAILABLE if not supported by the device
		/// @note Not available on all devices
		ErrorCode GetExtRegister(size_t const iRegID, std::string &oValue) noexcept;

		/// @brief Engage or release the brakes.
		/// This is available after a successful call to Init().
		/// @param[in] iStatus Status command
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if not supported by the device
		/// @note Not available on all devices (Achille-specific)
		ErrorCode ChangeBrakeStatus(BrakeStatus const iStatus) noexcept;

		/// @brief Get the current status of the brakes, whether engaged or released.
		/// This is available after a successful call to Init().
		/// @param[out] oStatus Status command
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if not supported by the device
		/// @note Not available on all devices (Achille-specific)
		ErrorCode GetBrakeStatus(BrakeStatus &oStatus) noexcept;

		/// @brief Get the status of the automaton.
		/// This is available after a successful call to Init().
		/// @param[out] oStatus Current automaton status
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode GetAutomatonStatus(AutomatonStatus &oStatus) noexcept;

		/// @brief Get the status of motor power.
		/// This is available after a successful call to Init().
		/// @param[out] oPowerStatus Current power status
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode GetPowerStatus(PowerStatus &oPowerStatus) noexcept;

		/// @brief Force the current Cartesian pose, overwriting the clutch offset.
		/// This is available after a successful call to StartCartesianPositionMode(). <br/>
		/// This call modifies the clutch offset.
		/// @param[in] iPose Cartesian pose to be returned by the next call to GetCartesianPose
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if iPose is not a valid Displacement <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartCartesianPositionMode has not been called before
		ErrorCode ForceCartesianPose(Displacement const &iPose) noexcept;

		/// @brief Force the current Cartesian pose, overwriting the clutch offset.
		/// This is available after a successful call to StartCartesianPositionMode(). <br/>
		/// This call modifies the clutch offset.
		/// @param[in] iPose Cartesian pose to be returned by the next call to GetCartesianPose
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if iPose is not a valid Transformation <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartCartesianPositionMode has not been called before
		ErrorCode ForceCartesianPose(Transformation const &iPose) noexcept;

		/// @brief Get the current clutch offset.
		/// This is available after a successful call to StartCartesianPositionMode().
		/// @param[out] oOffset Current clutch offset in the base frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartCartesianPositionMode has not been called before
		ErrorCode GetClutchOffset(Displacement &oOffset) noexcept;

		/// @brief Get the current clutch offset.
		/// This is available after a successful call to StartCartesianPositionMode().
		/// @param[out] oOffset Current clutch offset in the base frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartCartesianPositionMode has not been called before
		ErrorCode GetClutchOffset(Transformation &oOffset) noexcept;

		/// @brief Activate the clutch by software.
		/// This is available after a successful call to StartCartesianPositionMode().
		/// @param[in] iActivation True to activate, False to deactivate
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode ActivateClutch(const bool iActivation) noexcept;

		/// @brief Set a Cartesian pose setpoint.
		/// This is available after a successful call to StartCartesianPositionMode(). <br/>
		/// A call to SendSetpoints() is needed for the data to be effectively sent to the device.
		/// @param[in] iPose Cartesian pose setpoint in the world frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if iPose is not a valid Displacement <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartCartesianPositionMode has not been called before
		ErrorCode SetCartesianPose(Displacement const &iPose) noexcept;

		/// @brief Set a Cartesian pose setpoint.
		/// This is available after a successful call to StartCartesianPositionMode(). <br/>
		/// A call to SendSetpoints() is needed for the data to be effectively sent to the device.
		/// @param[in] iPose Cartesian pose setpoint in the world frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if iPose is not a valid Transformation <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartCartesianPositionMode has not been called before
		ErrorCode SetCartesianPose(Transformation const &iPose) noexcept;

		/// @brief Set a Cartesian pose setpoint and update the clutch offset.
		/// This is available after a successful call to StartCartesianPositionMode(). <br/>
		/// A call to SendSetpoints() is needed for the data to be effectively sent to the device.
		/// @param[in] iPose Cartesian pose setpoint in the world frame
		/// @param[in] iOffset Cartesian clutch offset to be added in the world frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if iPose or iOffset is not a valid Transformation <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartCartesianPositionMode has not been called before
		ErrorCode SetCartesianPoseWithClutchOffset(Displacement const &iPose, Displacement const &iOffset) noexcept;

		/// @brief Set a Cartesian speed setpoint.
		/// This is available after a successful call to StartCartesianPositionMode(). <br/>
		/// A call to SendSetpoints() is needed for the data to be effectively sent to the device.
		/// @param[in] iSpeed Cartesian speed setpoint in the world frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if iSpeed is not a valid vector <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartCartesianPositionMode has not been called before
		ErrorCode SetCartesianSpeed(CartesianVector const &iSpeed) noexcept;

		/// @brief Add a Cartesian force overlay.
		/// This is available after a successful call to StartCartesianPositionMode(). <br/>
		/// A call to SendSetpoints() is needed for the data to be effectively sent to the device.
		/// @param[in] iForce Cartesian force in the world frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_WRONGPARAMETER if iSpeed is not a valid vector <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartCartesianPositionMode has not been called before
		ErrorCode AddCartesianForceOverlay(CartesianVector const &iForce) noexcept;

		/// @brief Get the current Cartesian force applied by the device on the operator's hand.
		/// This is available after a successful call to StartCartesianPositionMode(). <br/>
		/// A call to ReadState() is needed to refresh the data.
		/// @param[out] oForce Current Cartesian force in the world frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartCartesianPositionMode has not been called before
		ErrorCode GetCartesianForce(CartesianVector &oForce) noexcept;

		/// @brief Set a joint angle setpoint.
		/// This is available after a successful call to StartJointPositionMode().
		/// A call to SendSetpoints() is needed for the data to be effectively sent to the device.
		/// @param[in] iJointAngles Joint angles setpoint in rad
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartJointPositionMode has not been called before
		ErrorCode SetJointAngles(JointVector const &iJointAngles) noexcept;

		/// @brief Set a joint speed setpoint.
		/// This is available after a successful call to StartJointPositionMode(). <br/>
		/// A call to SendSetpoints() is needed for the data to be effectively sent to the device.
		/// @param[in] iJointSpeeds Joint speeds setpoint in rad/s
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartJointPositionMode has not been called before
		ErrorCode SetJointSpeeds(JointVector const &iJointSpeeds) noexcept;

		/// @brief Add a joint torque overlay.
		/// This is available after a successful call to StartJointPositionMode(). <br/>
		/// A call to SendSetpoints() is needed for the data to be effectively sent to the device.
		/// @param[in] iJointTorques Joint torque overlay in Nm
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init or RaptorAPI::StartJointPositionMode has not been called before
		ErrorCode AddJointTorqueOverlay(JointVector const &iJointTorques) noexcept;

		/// @brief Get the status of an operator button.
		/// A call to ReadState() is needed to refresh the data.
		/// @param[in] iButton OperatorButton Identifier of the operator button
		/// @param[out] oButtonState State of the button (true for pressed/active)
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before or if button is not present on the device
		ErrorCode GetOperatorButton(OperatorButton const iButton, bool &oButtonState) noexcept;

		/// @brief Get the value of the trigger device.
		/// A call to ReadState() is needed to refresh the data.
		/// @param[out] oValue Current value of the trigger, as a float between 0 (open) and 1 (closed)
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before or if trigger is not present on the device
		ErrorCode GetTriggerValue(float32_t &oValue) noexcept;

		/// @brief Perform state update.
		/// @param[out] oTimeStamp The time of the current read state
		/// @param[in] iTimeout The timeout in milliseconds
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before,
		/// or in case of error on the communication channel
		ErrorCode ReadState(std::chrono::time_point<std::chrono::system_clock> &oTimeStamp, uint16_t const iTimeout = 0U) noexcept;

		/// @brief Perform setpoint update.
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if RaptorAPI::Init has not been called before,
		/// or in case of error on the communication channel
		ErrorCode SendSetpoints() noexcept;

		/// @brief Clear error in cartesian controller.
		/// This is available after a successfull call to Init()
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode ClearError() noexcept;

		/// @brief Switch the motor power supply on or off.
		/// This is available after a successful call to Init().
		/// @param[in] iEnableSupply True to switch On, false to switch Off
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if the power is managed by the front power button of the device <br/>
		/// E_EMERGENCYSTOP if an emergency stop button is pressed
		ErrorCode SwitchPowerOnOff(bool const iEnableSupply) noexcept;

		/// @brief Get the TCP offset of the robot with respect to the wrist.
		/// This is available after a successful call to Init().
		/// @param[out] oTcpOffset TCP offset as a displacement in the end effector frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode GetTCPOffset(Displacement &oTcpOffset) noexcept;

		/// @brief Change the TCP offset of the robot with respect to the wrist.
		/// This is available after a successful call to Init().
		/// @param[in] iTcpOffset TCP offset as a displacement in the end effector frame
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if not supported by the device
		ErrorCode ChangeTCPOffset(Displacement const &iTcpOffset) noexcept;

		/// @brief Set the trigger lock led to true or false.
		/// This is available after a successful call to Init(). <br/>
		/// A call to SendSetpoints() is needed for the data to be effectively sent to the device.
		/// @param[in] iStatus True to turn the led on, false to turn it Off
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if the device has no trigger lock led
		ErrorCode SetTriggerLockLed(bool const &iStatus) noexcept;

		/// @brief Check if the robot has reached its joints limits.
		/// This is available after a successful call to Init(). <br/>
		/// A call to ReadState() is needed to refresh the data before calling this function.
		/// @param[out] oLimitsReached Flags showing which joints have reached their limits
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode IsJointLimitReached(std::array<bool, MAX_NB_JOINTS> &oLimitsReached) noexcept;

		/// @brief  Get the value of a configuration parameter of type string.
		/// This is available after a successful call to Init().
		/// @param[in] iModuleName Name of the module
		/// @param[in] iParamName Name of the parameter
		/// @param[out] oValue Value of the parameter as std::string
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device <br/>
		/// E_NOTAVAILABLE if the parameter is not supported by the device <br/>
		/// E_WRONGPARAMETER if the type of the parameter is wrong
		ErrorCode GetParameterValue(std::string const &iModuleName, std::string const &iParamName, std::string &oValue) noexcept;

		/// @brief Define the clutching mode.
		/// This is available after a successful call to Init().
		/// @param[in] iMode Clutching mode
		/// @return E_NOERROR in case of success <br/>
		/// E_NOTCONNECTED if not connected to a manipulator device
		ErrorCode DefineClutchingMode(ClutchingMode const iMode) noexcept;

		/// @brief Get the number of joints.
		/// The number of joints is always defined, therefore this method cannot fail.<br/>
		/// All methods which are dependent on the number of joints, work on arrays with MAX_NB_JOINTS values,
		/// or else on a struct HAPTION::JointVector. Only the first N values are actually read/written,
		/// N being equal to the result of GetNbJoint().
		/// @return Number of joints of the force-feedback manipulator.
		size_t GetNbJoints() noexcept;

		/// @brief Change the anticipation factor for pose setpoints.
		/// In order to reduce the dragging force in free space, which is caused by
		/// the latency between the calls to RaptorAPI::ReadState and RaptorAPI::SendSetpoints,
		/// the RaptorAPI projects the pose setpoints based on the speed setpoints (first-order anticipation). <br/>
		/// The anticipation is applied in Cartesian space or in joint space, depending on the active mode.<br/>
		/// This is available after a successful call to Init().
		/// @param[in] iFactor Anticipation factor to be used, between 0.0 and 1.0 <br/>
		/// <ul><li> 0.0 means no anticipation at all </li>
		/// <li> 1.0 means projection on the full time delay between RaptorAPI::ReadState and RaptorAPI::SendSetpoints </li></ul>
		/// The default value is 0.5, i.e. 50% of the measured time delay.
		/// @return E_NOERROR in case of success <br/>
		/// E_WRONGPARAMETER if the parameter is not valid (e.g. negative or above 1.0)
		ErrorCode ChangeAnticipationFactor(float32_t const iFactor) noexcept;

		/// @brief Define the height of the operator.
		/// Quotation from <b> ISO/TS 15066:2016(en) Robots and robotic devices - Collaborative robots </b>:
		/// "This Technical Specification provides guidance for collaborative robot operation where a robot system and people
		/// share the same workspace. In such operations, the integrity of the safety-related control system is of major importance,
		/// particularly when process parameters such as speed and force are being controlled." <br/>
		/// Following the guidelines of the standard hereabove, the RaptorAPI reduces the speed of motion when the
		/// device handles reaches the height of i) the torso and ii) the head of the operator. <br/>
		/// The relative heights of torso and head are estimated based on the height of the operator. <br/>
		/// This is available after a successful call to Init().
		/// @param[in] iHeight Height of the operator in meters <br/>
		/// The height is mesured between the lower base of the force-feedback manipulator and the top of the operator head. <br/>
		/// In case the operator is standing, it's the operator height minus the table height. <br/>
		/// In case the operator is sitting, then the height should be measured. <br/>
		/// The default value is 0.9 meters, therefore assuming the operator is standing and the manipulator is fixed on a standard table.
		/// @return E_NOERROR in case of success <br/>
		/// E_WRONGPARAMETER if the parameter is not valid (e.g. negative or above 2.0)
		ErrorCode DefineOperatorHeight(float32_t const iHeight) noexcept;

	private:
		// @brief Internals
		RaptorAPIInternals *m_internals{nullptr};
	};

}

#endif /// H_RAPTOR_API