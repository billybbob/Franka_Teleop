This project includes a basic implementation of test programs (TestCalibration, TestImpedance and TestAdmittance) using ROS2 and RaptorAPI.
This guide is written for Linux; Windows users need to adapt it in the appropriate way.

## Installation and setup
1. Installation of ros2: 
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
2. Installation of colcon: 
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#install-colcon
3. [Optional] Clone as a workspace:
If you want to follow the ros2 way for creating a workspace, you can clone this repository under ~/ros2_ws/, or visit the documentation page for creating a worspace https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
NOTE: the procedure described here has been validated WITHOUT creating a workspace!
4. Copy the RaptorAPI headers into "src/haption_raptor_api/Dependencies/RaptorAPI", overwriting the old files
5. Copy the RaptorAPI shared libraries to "src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-<version>"
6. Make sure that the ".param" file for your device is accessible

## Usage
0. For newbies: run "local_setup.bash" first! (typically under "/opt/ros/<version>"). Example: "source /opt/ros/humble/local_setup.bash".

In a first terminal:
1. Set LD_LIBRARY_PATH so that it points to "src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-<version>"
2. Run "local_setup.bash"
3. Prepare the raptor_api_interfaces:
	colcon build --packages-select raptor_api_interfaces
4. Start the RaptorAPIWrapper node by calling ./start_RaptorAPIWrapper.sh

In a second terminal:
1. Run "local_setup.bash"
2. Calibrate the robot if it was not already calibrated:
    - Edit the file "src/test_calibration/parameters.yaml" according to your network setup (local_ip_address, ff_device_ip_address, ff_device_param_file)
    - Run the calibration node by calling ./start_TestCalibration
	- Follow the calibration instructions given in the first terminal
	- Stop with CRTL-C
3. Run the impedance node:
    - Edit the "src/test_impedance/parameters.yaml" according to your network setup (local_ip_address, ff_device_ip_address, ff_device_param_file)
    - Call ./start_TestImpedance
	- When you see info messages sarting with "Status:", push the power button of the force-feedback device, take the handle and check the haptic plane
	- Stop with CTRL-C
4. Run the admittance node:
    - Edit the file "src/test_admittance/parameters.yaml" according to your network setup (local_ip_address, ff_device_ip_address, ff_device_param_file)
    - Call ./start_TestAdmittance
	- When you see info messages sarting with "Status:", push the power button of the force-feedback device, take the handle and check the haptic plane
	- Stop with CTRL-C
