#!/bin/bash

if [ -n "$3" ];then
    echo -e "\n***APT-PROXY***"
	sh -c 'echo "Acquire::http { Proxy \"'$3'\"; };" > /etc/apt/apt.conf.d/01proxy'
	cat /etc/apt/apt.conf.d/01proxy
fi

echo -e "\n***UPDATE***"
cat /etc/apt/sources.list
apt-get update
echo -e "\n***UPGRADE***"
apt-get dist-upgrade -y
echo -e "\n***INSTALL HELPER***"
apt-get install -y \
    python-setuptools ccache wget curl curl-ssl sudo git-buildpackage dput \
    python-yaml python-pip python-support python-apt git-core mercurial subversion \
    python-all gccxml python-empy python-nose python-mock python-minimock \
    lsb-release python-numpy python-wxgtk2.8 python-argparse python-networkx \
    graphviz python-sphinx doxygen python-epydoc cmake pkg-config openssh-client python-paramiko \
    cppcheck x11-utils

echo -e "\n***GET KEY***"
wget http://packages.ros.org/ros.key -O - | apt-key add -
echo -e "\n***WRITE SOURCE***"
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu '$1' main" > /etc/apt/sources.list.d/ros-latest.list'
cat /etc/apt/sources.list.d/ros-latest.list

echo -e "\n***UPDATE***"
apt-get update
echo -e "\n***INSTALL ROS***"
apt-get install -y ros-$2-ros
echo -e "\n***INSTALL BASIC ROS PACKAGES***"
apt-get install -y python-rosinstall python-rosdistro python-rosdep python-rospkg python-catkin-pkg
