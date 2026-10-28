#!/bin/bash
export ROS_DISCOVERY_SERVER="172.23.16.88:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4_discovery/super_client_configuration_2.xml"
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID=2

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tp1_teach_repeat teach.launch.py \
  use_sim_time:=false \
  mode:=mixed \
  route_file:=routes/lab_route.json \
  overwrite:=true \
  publish_stamped:=false \
  params_file:="../params_tb4.yaml" \
  scan_topic:=/tb4_2/scan \
  joint_states_topic:=/tb4_2/joint_states \
  cmd_vel_topic:=/tb4_2/cmd_vel
