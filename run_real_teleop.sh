#!/usr/bin/env bash
set -e

export ROS_DISCOVERY_SERVER="172.23.16.88:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4_discovery/super_client_configuration_2.xml"
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash

echo "🎮 ==================================================="
echo "🎮 TELEOPERACION ROBOT 2 (/tb4_2/cmd_vel)"
echo "🎮 Controles: i (avanzar), j (girar izq), l (girar der), k (frenar)"
echo "🎮 ==================================================="

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/tb4_2/cmd_vel
