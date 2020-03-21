#!/bin/bash

# Project Directory as an argument
dir=$1
cd $dir
source /opt/ros/${ROS_DISTRO}/setup.sh

catkin_make