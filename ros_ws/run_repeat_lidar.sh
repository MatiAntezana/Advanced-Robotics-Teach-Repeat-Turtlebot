#!/bin/bash
ROBOT_ID="${ROBOT_ID:-1}"
turtlebot_mode "$ROBOT_ID"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tp1_teach_repeat repeat.launch.py \
  use_sim_time:=false \
  mode:=lidar \
  route_file:=routes/lab_route.json \
  publish_stamped:=false \
  params_file:="$PWD/../params_tb4.yaml" \
  --ros-args -r /scan:=/tb4_${ROBOT_ID}/scan -r /joint_states:=/tb4_${ROBOT_ID}/joint_states -r /cmd_vel:=/tb4_${ROBOT_ID}/cmd_vel -r /tf:=/tb4_${ROBOT_ID}/tf -r /tf_static:=/tb4_${ROBOT_ID}/tf_static
