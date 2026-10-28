#!/bin/bash
ROBOT_ID="${ROBOT_ID:-2}"
turtlebot_mode "$ROBOT_ID"
ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}'
