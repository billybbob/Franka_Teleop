#!/bin/bash

# Clone Franka dependencies into the workspace
if [ ! -d /ros2_ws/src/franka_description ] && [ ! -d /ros2_ws/src/libfranka ]; then
  vcs import --recursive /ros2_ws/src < /ros2_ws/src/franka.repos
fi

exec "$@"