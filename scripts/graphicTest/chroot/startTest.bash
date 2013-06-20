#!/bin/bash
source /opt/ros/electric/setup.bash
export ROS_PACKAGE_PATH=~/ros:$ROS_PACKAGE_PATH
rostest navigation_test navigation.test robot:=cob3-3 robot_env:=ipa-kitchen route:=1
