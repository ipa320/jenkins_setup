#!/bin/bash
mkdir -p ~/ros; cd ~/ros
git clone https://github.com/SimonEbner/ba

source /opt/ros/electric/setup.bash
export ROS_PACKAGE_PATH=~/ros:$ROS_PACKAGE_PATH
mkdir -p ~/.ros; chmod -R a+wr ~/.ros
echo $ROS_PACKAGE_PATH
rosmake --rosdep-install --rosdep-yes navigation_test
