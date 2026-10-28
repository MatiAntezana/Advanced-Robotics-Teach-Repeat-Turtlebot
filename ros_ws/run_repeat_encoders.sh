#!/bin/bash
turtlebot_mode 2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tp1_teach_repeat repeat.launch.py \
  use_sim_time:=false \
  mode:=encoders \
  route_file:=routes/lab_route.json \
  publish_stamped:=false \
  params_file:="$PWD/../params_tb4.yaml" \
  --ros-args -r /scan:=/tb4_2/scan -r /joint_states:=/tb4_2/joint_states -r /cmd_vel:=/tb4_2/cmd_vel
