#!/bin/sh

echo "\n***APT-PROXY***"
sh -c 'echo "Acquire::http { Proxy "http://cob-jenkins-server:3142"; };" > /etc/apt/apt.conf.d/01proxy'
echo "\n***UPDATE***"
cat /etc/apt/sources.list
apt-get update
echo "\n***UPGRADE***"
apt-get dist-upgrade -y
echo "\n***INSTALL HELPER***"
apt-get install -y \
    python-setuptools ccache wget curl curl-ssl sudo git-buildpackage dput \
    python-yaml python-pip python-support git-core mercurial subversion \
    python-all gccxml python-empy python-nose python-mock python-minimock \
    lsb-release python-numpy python-wxgtk2.8 python-argparse python-networkx \
    graphviz python-sphinx doxygen python-epydoc cmake pkg-config

echo "\n***GET KEY***"
wget http://packages.ros.org/ros.key -O - | apt-key add -
echo "\n***WRITE SOURCE***"
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu '$1' main" > /etc/apt/sources.list.d/ros-latest.list'
cat /etc/apt/sources.list.d/ros-latest.list

echo "\n***UPDATE***"
apt-get update
echo "\n***INSTALL ROS***"
apt-get install -y ros-$2-ros
echo "\n***INSTALL ROSINSTALL***"
pip install -U rosinstall
