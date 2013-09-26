#!/bin/bash
source /opt/ros/electric/setup.bash
export ROS_PACKAGE_PATH=~/ros:$ROS_PACKAGE_PATH
VGL=/opt/VirtualGL/bin/vglrun
$VGL rosmake --test-only navigation_test
