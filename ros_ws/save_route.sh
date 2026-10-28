#!/bin/bash
turtlebot_mode 2
ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}'
