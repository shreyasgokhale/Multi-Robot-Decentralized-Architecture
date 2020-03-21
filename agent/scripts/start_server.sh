#!/bin/bash

# Project Directory as an argument
dir=$1
cd $dir
source /opt/ros/${ROS_DISTRO}/setup.sh
source dir/install_isolated/setup.sh
roslaunch cartographer_ros agent_cartographer.launch