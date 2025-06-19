#!/bin/bash
source /opt/ros/humble/setup.bash
colcon build --packages-select haption_raptor_api
. install/local_setup.bash 
ros2 run haption_raptor_api raptor_api_wrapper --sched SCHED_FIFO --priority 80
