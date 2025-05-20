#!/bin/bash
source /opt/ros/humble/setup.bash

colcon build --packages-select test_admittance
. install/local_setup.bash 
ros2 run test_admittance test_admittance --ros-args --params-file ./src/test_admittance/parameters.yaml
