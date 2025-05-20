/*!
* @file RobotTypes.hpp
* This file contains complex type and enum definitions, within the HAPTION namespace.
* @copyright Haption SA
*/

#ifndef H_ROBOT_TYPES
#define H_ROBOT_TYPES

#include <array>
#include "Types.hpp"

namespace HAPTION
{
	uint16_t constexpr MAX_NB_JOINTS { 16U };

	uint32_t constexpr SOFT_VERSION{ 20201231U };

	/// @brief Simple Displacement.
	/// Translation in meters (m).
	/// Rotation as a unit quaternion.
	struct Displacement
	{
	public:
		/// @brief Position value on X in world coordinates, in meters (m)
		float32_t t_x;
		/// @brief Position value on Y in world coordinates, in meters (m)
		float32_t t_y;
		/// @brief Position value on Z in world coordinates, in meters (m)
		float32_t t_z;
		/// @brief "i" value of the orientation quaternion
		float32_t q_x;
		/// @brief "j" value of the orientation quaternion
		float32_t q_y;
		/// @brief "k" value of the orientation quaternion
		float32_t q_z;
		/// @brief Scalar value of the orientation quaternion
		float32_t q_w;
	};

	/// @brief Simple transformation matrix.
	/// Translation in meters (m).
	/// The rotation is stored row-major, and there is no scale:\n
	/// <table>
	/// <tr><td>r_11</td><td>r_12</td><td>r_13</td><td>t_x</td></tr>
	/// <tr><td>r_21</td><td>r_22</td><td>r_23</td><td>t_y</td></tr>
	/// <tr><td>r_31</td><td>r_32</td><td>r_33</td><td>t_z</td></tr>
	/// <tr><td>0</td><td>0</td><td>0</td><td>1</td></tr>
	/// </table>
	typedef struct
	{
		float32_t r_11;
		float32_t r_12;
		float32_t r_13;
		float32_t r_21;
		float32_t r_22;
		float32_t r_23;
		float32_t r_31;
		float32_t r_32;
		float32_t r_33;
		float32_t t_x;
		float32_t t_y;
		float32_t t_z;
	} Transformation;

	/// @brief Simple Cartesian Vector
	/// This is used to store speeds and forces/torques.
	/// In the case of a speed, the translation is in m/s and the rotation in rad/s.
	/// In the case of a force/torque, the translation is in N and the rotation in Nm.
	typedef struct
	{
		float32_t t_x;
		float32_t t_y;
		float32_t t_z;
		float32_t r_x;
		float32_t r_y;
		float32_t r_z;
	} CartesianVector;

	/// @brief Simple Vector, used for joint mode
	/// This is used to store joint angles, speeds and torques.
	/// In the case of joint angles, the values are in rad.
	/// In the case of joint speeds, the values are in rad/s.
	/// In the case of joint torques, the values are in Nm.
	typedef struct
	{
		std::array<float32_t, MAX_NB_JOINTS> v;
	} JointVector;

	/// @brief Error codes.
	enum class ErrorCode : uint16_t
	{
		/// No error (0)
		E_NOERROR,
		/// Fatal error: not connected to the device (1)
		E_NOTCONNECTED,
		/// Fatal error: software watchdog on the connection raised (2)
		E_WATCHDOG,
		/// Fatal error: related to memory allocation (3)
		E_MEMORYALLOCATION,
		/// Fatal error: hardware breakdown (4)
		E_BREAKDOWN,
		/// Non-fatal error: movement stopped due to deadman switch released (5)
		E_SAFETYSTOP_DEADMAN,
		/// Non-fatal error: movement stopped due to power button depressed (6)
		E_SAFETYSTOP_POWERBUTTON,
		/// Non-fatal error: movement stopped due to emergency stop pressed (7)
		E_EMERGENCYSTOP,
		/// Non-fatal error: motor torques reduced because of exceeded rotor temperature threshold (8)
		E_TEMPERATURE,
		/// Non-fatal error: movement stopped due to excessive speed (9)
		E_SPEEDEXCESS,
		/// Non-fatal error: timeout on receiving an update message (10)
		E_TIMEOUT,
		/// Non-fatal error: no movement since the current position is higher than upper limit (11)
		E_BEYONDUPPER,
		/// Non-fatal error: no movement since the current position is lower than lower limit (12)
		E_BEYONDLOWER,
		/// Programming error: function not available in this context (13)
		E_NOTAVAILABLE,
		/// Programming error: wrong parameter given (14)
		E_WRONGPARAMETER,
		/// Programming error: wrong serial number given (15)
		E_WRONGSERIALNUMBER,
		/// Programming error: file not found (16)
		E_FILENOTFOUND,
		/// Ping error (17)
		E_PINGERROR,
		/// License error (18)
		E_LICENSEERROR,
		/// Programming error: wrong joint gain given (19)
        E_WRONGJOINTGAINS,
        /// Programming error: wrong cartesian gain given (20)
        E_WRONGCARTESIANGAINS,
		/// Fatal error: Device firmware not ready (21)
        E_FIRMWARENOTREADY,
        /// Other error (21)
        E_OTHER
	};

	/// @brief Power status.
	/// This status can be accessed by calling RaptorAPI::GetPowerStatus().
	enum class PowerStatus : uint16_t
	{
		/// No power on the motor H-bridge
		P_NOPOWER,
		/// Power on the motor H-bridge, but H-bridge closed
		P_POWER_INHIBITED,
		/// Power on the motor H-bridge, and H-bridge open
		P_POWER_DISINHIBITED,
		/// Emergency stop
		P_POWER_EMERGENCY_STOP
	};

	/// @brief Detailed motor status.
	/// This status can be accessed by calling RaptorAPI::GetMotorStatus().
	struct MotorStatus
	{
		/// Calibration status (when set, actuator is calibrated)
		uint16_t calibrated : 1;
		/// PWM inhibition flag (when set, the PWM is inhibited)
		uint16_t inhibition : 1;
		/// I2T warning (over-temperature of the robot, causing a reduction of torque)
		uint16_t i2t : 1;
		/// Encoder error (breakdown or bus error)
		uint16_t encoderError : 1;
		/// Current error (current drag error)
		uint16_t currentError : 1;
		/// PWM error (breakdown of the H-bridge)
		uint16_t pwmError : 1;
		/// PWM overheat error (temperature of the H-bridge)
		uint16_t pwmOverheatError : 1;
		/// Checksum error sent from DSP (protocol error on internal bus)
		uint16_t checksumDSPError : 1;
		/// Checksum error calculated by uC (protocol error on internal bus)
		uint16_t checksumCalculatedError : 1;
		/// Encoder mismatch (when two encoders are present on the motor/joint)
		uint16_t encoderMismatchError : 1;
		/// Motor over speed (Motor speed is higher than limit)
		uint16_t overSpeed : 1;
		/// ConnectionError (CAN error)
		uint16_t connectionError : 1;
		/// MechanicalError (spring compensation error)
		uint16_t mechanicalError : 1;
		/// Drag error
		uint16_t dragError : 1;
		/// Discontinuity error
		uint16_t discontinuityError : 1;
	};

	/// @brief Detailed sensor status.
	/// This status can be accessed by calling RaptorAPI::GetSensorStatus().
	struct SensorStatus
	{
		/// Fatal error (mechanical breakdown)
		uint8_t fatalError : 1;
		/// CRC error (bus error)
		uint8_t crcError : 1;
		/// Warning (potential mechanical issue)
		uint8_t warning : 1;
		/// Sensor reading is out of mechanical range
		uint8_t outOfRange : 1;
	};

	/// @brief Operator buttons.
	/// Those values designate all input devices available to the operator on the Virtuose device.
	/// The code can query the current status of the input devices with RobotAPI::GetOperatorButton().
	enum class OperatorButton : uint16_t
	{
		/// Front power button (true means pressed)
		B_POWERBUTTON,
		/// Dead-man switch (handle or pedal, true means active)
		B_DEADMANSWITCH,
		/// Clutch button (true means clutching is effective)
		B_CLUTCH,
		/// Trigger lock lever on the TREX handle (true means pushed)
		B_TRIGGERLOCK,
		/// "Next" button on the TAO button box (true means pushed)
		B_NEXT,
		/// "Rotation +" button on the TREX handle (true means pushed)
		B_ROTPLUS,
		/// "Rotation -" button on the TREX handle (true means pushed)
		B_ROTMINUS,
		/// "Stop" button on the TAO button box (true means pushed)
		B_STOP,
		/// "Wait" button on the TAO button box (true means pushed)
		B_WAIT,
		/// "Redundancy +" button on the TAO button box (true means pushed)
		B_ELBOWPLUS,
		/// "Redundancy -" button on the TAO button box (true means pushed)
		B_ELBOWMINUS,
		/// "Precision +" button on the TAO button box (true means pushed)
		B_PRECISIONPLUS,
		/// "Precision -" button on the TAO button box (true means pushed)
		B_PRECISIONMINUS,
		/// Left button on the COBRA handle (true means pushed)
		B_LEFT,
		/// Right button on the COBRA handle (true means pushed)
		B_RIGHT,
		/// "Force high" switch position on the TAO button box (true means active)
		B_FORCEHIGH,
		/// "Force low" switch position on the TAO button box (true means active)
		B_FORCELOW,
		/// "Force medium" switch position on the TAO button box (true means active)
		B_FORCEMEDIUM,
		/// "External signal 1" first signal on the optional connector
		B_EXTSIGNAL1,
		/// "External signal 2" second signal on the optional connector
		B_EXTSIGNAL2
	};

	/// @brief Calibration status.
	/// This status can be accessed by calling RobotAPI::GetCalibrationStatus().
	enum class CalibrationStatus : uint16_t
	{
		/// Initial status
		C_IDLE,
		/// Waiting for the operator to push the power button
		C_WAITINGFORPOWER,
		/// Currently moving towards the joint limits
		C_MOVING,
		/// Waiting for the user to move to the calibration position and push the calibration button
		C_WAITINGFORUSER,
		/// Calibration positions are sent to the robot
		C_CALIBRATIONSENT,
		/// Successfully calibrated
		C_CALIBRATED,
		/// Failed to calibrate
		C_CALIBRATIONFAILED
	};

	/// @brief Calibration type
	enum class CalibrationType : uint16_t
	{
		/// Automatic calibration, i.e. the device moves on its own until it reaches its joint limits
		AUTOMATIC_CALIBRATION,
		/// Manual calibration, i.e. an action of the operator is required for calibration
		MANUAL_CALIBRATION,
		/// Absolute calibration, i.e. calibration is done using absolute encoders or potentiometers
        ABSOLUTE_CALIBRATION
	};

	/// @brief TAO button box.
	enum class ButtonsBox : uint16_t
	{
		/// TAO button box is not present on the robot
		BB_NOTPRESENT = 0,
		/// TAO button box is present on the robot
		BB_PRESENT = 1
	};

	/// @brief State machine status.
	/// This can be accessed by calling RobotAPI::GetAutomatonStatus().
	enum class AutomatonStatus : uint16_t
	{
		/// Unknown state, machine not running
		A_NONE,
		/// Waiting for calibration to be performed
		A_CALIBRATION,
		/// Waiting for activation command
		A_IDLE,
		/// Waiting for operator to press the power button
		A_WAIT_POWER,
		/// Waiting for operator to activate the deadman-switch
		A_INACTIVE,
		/// Moving to the start pose
		A_HOMING,
		/// Fully active
		A_ACTIVE,
		/// Non-fatal error, can be solved by depressing the power button
		A_ERROR,
		/// Fatal error
		A_BREAKDOWN
	};

	/// @brief Connector Errors.
	enum class ConnError : uint16_t
	{
		/// No error (0)
		CE_NOERROR,
		/// Manipulator Communication Error
		CE_MANIPULATORCONNECTION,
		/// Robot Communication Error
		CE_ROBOTCONNECTION,
		/// Robot mode change error
		CE_ROBOTMODECHANGE,
		/// Manipulator wrong serial number error
		CE_MANIPULATORWRONGSN,
		/// Manipulator EmergencyStop error
		CE_MANIPULATOREMERGENCYSTOP,
		/// Manipulator breakdown error
		CE_MANIPULATORBREAKDOWN,
		/// Robot breakdown error
		CE_ROBOTBREAKDOWN,
		/// Manipulator speed excess error
		CE_SPEEDEXCESS,
		/// Manipulator discontinuity error
		CE_MANIPULATORDISCONTINUITY,
		/// Robot discontinuity error
		CE_ROBOTDISCONTINUITY,
		/// Manipulator drag error
		CE_MANIPULATORDRAG,
		/// Robot drag error
		CE_ROBOTDRAG,
		/// Robot beyond upper error
		CE_ROBOTBEYONDUPPER,
		/// Robot beyond lower error
		CE_ROBOTBEYONDLOWER,
		/// Application Error
		CE_APPFAILED,
		// Robot failed
		CE_COMSLAVEFAILED,
		// Manipulator failed
		CE_COMMASTERFAILED,
		// Initialization failed
		CE_INITFAILED
	};

	/// @brief Communication Errors
	enum class CommunicationError : uint16_t
	{
		// No error
		CME_NOERROR,
		// Invalid address
		CME_INVALID_ADDRESS,
		// Socket errors
		CME_SOCKET_ERROR,
		CME_SOCKET_ENOTSOCK,
		CME_SOCKET_ECONNRESET,
		CME_SOCKET_ENOTCONN,
		CME_SOCKET_TIMEDOUT,
		CME_SOCKET_NOTINITIALISED,
		// Not found: file, asset...etc
		CME_NOT_FOUND,
		// Wrong target
		CME_WRONG_TARGET,
		// Data not available
		CME_NOTAVAILABLE

	};

	// @brief enumeration of the input parameters types used
	enum class UDPChannelError : uint16_t
	{
		UCE_NOERROR,
		UCE_DLL_ERROR,
		UCE_FREE_LIB_ERROR,
		UCE_RESERVE_CHANNEL_ERROR,
		UCE_OPEN_CHANNEL_ERROR,
		UCE_SEND_MSG_ERROR,
		UCE_RECV_MSG_ERROR,
		UCE_CONNECTION_FAILED,
		UCE_UNTRUSTED_CONNECTOR
	};

	// @brief Status of the brakes
	// @note Not available on all devices (Achille-specific)
	enum class BrakeStatus : uint16_t
	{
		BRAKE_ENGAGED,
		BRAKE_RELEASED
	};

	/// @brief Clutching mode.
	/// This mode can be changed by calling RaptorAPI::SetClutchingMode().
	enum class ClutchingMode : uint16_t
	{
		/// Clutching active on all movements, force cancelled
		CM_CLUTCHING_ALL_NOFORCE,
		/// Clutching active on all movements, force maintained
		CM_CLUTCHING_ALL_FORCE,
		/// Clutching active on translation movements, force cancelled
		CM_CLUTCHING_TRANS_NOFORCE,
		/// Clutching active on translation movements, force maintained
		CM_CLUTCHING_TRANS_FORCE,
		/// Clutching active on rotation movements, force cancelled
		CM_CLUTCHING_ROT_NOFORCE,
		/// Clutching active on rotation movements, force maintained
		CM_CLUTCHING_ROT_FORCE,
		/// Clutching inactive
		CM_CLUTCHING_NONE
	};

	/// @brief TREX - IFS coupling modality status.
	enum class IFSCouplingModalities : uint16_t
	{
		/// TREX is coupled in Cartesian Mode and the digital twin in IFS follows 
		/// the real robot, in Cartesian for the EE and in angular for the joints
		IFS_MANIP_ROBOT,
		/// Monitoring: the real robot follows the trajectory of 
		/// the digital one; TREX acts only as a gateway
		IFS_IPSI_ROBOT,
		/// The digital twin is controlled in Cartesian mode at the EE from the 
		/// Virtuose and in angular mode from the real robot ; the real robot 
		/// follows the movements of the digital one ; the Virtuose receives 
		/// the Cartesian position of the digital twin EE. 
		IFS_MANIP_IPSI_ROBOT,
		/// The digital twin is controlled in Cartesian mode at the EE from the 
		/// Virtuose, but not in articular from the real robot ; the real robot 
		/// *only* follows the movements of the digital one ; the Virtuose receives 
		/// the Cartesian position of the digital twin EE. 
		IFS_MANIP_IPSI_ROBOT_SIM
	};
} // namespace HAPTION

#endif /// H_DRAFT_ROBOT_TYPES
