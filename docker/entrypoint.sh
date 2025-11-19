#!/bin/bash

set -e

export ROS2_INSTALL_PATH="/opt/ros/humble"
export ROS_NAMESPACE="leo05"
export ROS_DOMAIN_ID=0

# setup ros2 environment
cd /root/ros2_ws
source "${ROS2_INSTALL_PATH}/setup.bash"
colcon build --symlink-install
source "/root/ros2_ws/install/setup.bash"

exec "$@"
