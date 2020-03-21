#!/bin/bash

ROBOT_PREFIX=${1:-mira}

source /simulator/catkin_ws/devel/setup.bash
# shellcheck disable=SC1091
cd /simulator/catkin_ws/ && catkin_make
sleep 10s

source /simulator/catkin_ws/devel/setup.bash
#sleep 10s

cd /simulator/catkin_ws/src/multiple-agents-launcher/launch && roslaunch multi_turtlebot3_modified.launch
#sleep 20m