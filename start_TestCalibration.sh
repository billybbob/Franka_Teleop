#!/bin/bash
source /opt/ros/humble/setup.bash

colcon build --packages-select test_calibration
. install/local_setup.bash 
ros2 run test_calibration test_calibration --ros-args --params-file ./src/test_calibration/parameters.yaml
