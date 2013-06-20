#!/bin/bash
export ROS_PACKAGE_PATH=~/ros:$ROS_PACKAGE_PATH
export DISPLAY=:0
rostest navigation_test navigation.test robot:=cob3-3 robot_env:=ipa-kitchen route:=1
