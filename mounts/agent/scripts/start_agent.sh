#!/bin/bash
# Project Directory as an argument
dir_robot=${1:-/catkin_robot}
dir_cartographer=${1:-/catkin_ws}
source /opt/ros/${ROS_DISTRO}/setup.sh

# Catkin make cartographer
cd $dir_cartographer
catkin_make_isolated --install --use-ninja --cmake-args -DBUILD_GRPC=ON

# Catkin make the robot packages
cd $dir_robot
catkin_make
source $dir_robot/devel/setup.sh

source $dir_cartographer/devel_isolated/setup.sh 

export ROS_PACKAGE_PATH=$dir_robot/src:$dir_cartographer/source:$ROS_PACKAGE_PATH

# source $dir_cartographer/devel_isolated/setup.sh 
export ROS_MASTER_URI=http://localhost:11311
roscore -p 11311 &
sleep 3s
roscore -p 11811 &

roslaunch agent-starter cartographer.launch &
sleep 3s
source $dir_robot/devel/setup.sh

sleep 3s
roslaunch agent-starter nimbro.launch \
    simulator_nimbro_target:=${SIMULATOR_NIMBRO_TARGET} \
    simulator_nimbro_port:=${SIMULATOR_NIMBRO_PORT} \
    robot_name:=${ROBOT_NAME} &

sleep 1d
