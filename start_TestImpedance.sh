#!/bin/bash
source /opt/ros/humble/setup.bash

colcon build --packages-select test_impedance
. install/local_setup.bash 
ros2 run test_impedance test_impedance --ros-args --params-file ./src/test_impedance/parameters.yaml
