#!/bin/bash
mkdir -p ~/ros
# for debugging purposes only
apt-get -y install python-pip git-core
export PATH=/usr/local/bin:$PATH
type rosinstall > /dev/null 2>&1 || pip install -U rosinstall

rosinstall ~/ros $DIR/setup.rosinstall
cd ~/ros
git clone git://github.com/SimonEbner/ba.git

apt-get install -y ros-electric-care-o-bot ros-electric-desktop-full ros-electric-simulator-gazebo
apt-get install -y mysql-common libmysqlclient16 libmysqlclient-dev libmysqlclient16-dev
apt-get install -y libwxgtk2.8-0 libwxgtk2.8-dev python-wxgtk2.8 python-wxversion wx2.8-headers

source ~/.bashrc
source /opt/ros/electric/setup.bash
export ROS_PACKAGE_PATH=~/ros:$ROS_PACKAGE_PATH
rosmake --rosdep-install navigation_test

dpkg -l > $WORKSPACE/dpkg_output.log

export DISPLAY=:0
rostest navigation_test navigation.test robot:=cob3-3 robot_env:=ipa-kitchen route:=1

rosrun rosunit clean_junit_xml.py
