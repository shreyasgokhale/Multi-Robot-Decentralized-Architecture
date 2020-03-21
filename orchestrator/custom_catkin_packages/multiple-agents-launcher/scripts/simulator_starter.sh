#!/bin/bash

# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

ROBOT_COUNT=${1:-3}
SIMULATOR_THREAD_COUNT=${2:-4}
ROBOT_PREFIX=${3:-mira}

# shellcheck disable=SC1091
source /simulator/catkin_ws/devel/setup.bash
roslaunch multi_turtlebot3_modified.launch

#sleep 20m

#roslaunch turtlebot3_gazebo gui:=false multi_turtlebot3.launch  
# # shellcheck disable=SC1091
# source /root/catkin_ws/src/argos_bridge/scripts/setupArgosEnv.sh "$ROBOT_PREFIX" "$ROBOT_COUNT" "$SIMULATOR_THREAD_COUNT" "$SIMULATOR_CALLBACK_TIMEOUT"
# /root/catkin_ws/src/argos_bridge/scripts/runArgosSim.sh "${ARGOS_WORLD_FILE}"
