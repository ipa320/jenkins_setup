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
    python-setuptools ccache wget curl sudo git-buildpackage dput \
    python-yaml python-pip python-support python-apt git-core mercurial subversion \
    python-all gccxml python-empy python-nose python-mock python-minimock \
    lsb-release python-numpy python-wxgtk2.8 python-argparse python-networkx \
    graphviz python-sphinx doxygen python-epydoc cmake pkg-config openssh-client python-paramiko \
    cppcheck x11-utils

saucy='saucy'
if [ $1 != $saucy ]
then
    apt-get install -y curl-ssl
fi

echo -e "\n***GET KEY***"
wget http://packages.ros.org/ros.key -O - | apt-key add -
echo -e "\n***WRITE SOURCE***"
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu '$1' main" > /etc/apt/sources.list.d/ros-latest.list'
cat /etc/apt/sources.list.d/ros-latest.list

sh -c 'echo "deb http://doc.openrobotino.org/download/packages/amd64 ./" > /etc/apt/sources.list.d/robotino.list'
cat /etc/apt/sources.list.d/robotino.list

echo -e "\n***UPDATE***"
apt-get update
echo -e "\n***INSTALL ROS***"
apt-get install -y ros-$2-ros


precise='precise'
if [ $1 = $precise ]    # string1 has not been declared or initialized.
then
    echo -e "\n***PYPASSING BROKEN DEPENDENCY - INSTALL python-wstool***"

    echo -e "\n### INSTALLING ROS ###"
    apt-get install -y python-vcstools python-rospkg python-routes python-rosdistro python-ropemacs python-rosinstall-generator python-rope python-rosrelease python-rosdep python-roman
    echo -e "\n### DOWNLOAD python-wstool ###"
    apt-get download -y python-wstool

    echo -e "\n### REMOVING BROKEN DEPENDECY IN PACKAGE python-wstool AND INSTALLING IT ###"
    dpkg-deb -x python-wstool_0.1.4-1_all.deb changedeb
    dpkg-deb --control python-wstool_0.1.4-1_all.deb changedeb/DEBIAN
    mv changedeb/DEBIAN/control changedeb/DEBIAN/control_old
    cat changedeb/DEBIAN/control_old | sed -e 's/, python:any (>= 2.7.1-0ubuntu2)//g' > changedeb/DEBIAN/control
    rm changedeb/DEBIAN/control_old
    dpkg -b changedeb python-wstool_0.1.4-1_all_rmdeb.deb
    rm -rf changedeb/
    rm python-wstool_0.1.4-1_all.deb
    dpkg -i python-wstool_0.1.4-1_all_rmdeb.deb
    rm python-wstool_0.1.4-1_all_rmdeb.deb

    echo -e "\n### INSTALLING python-rosinstall ###"
    apt-get install -y python-rosinstall
fi

echo -e "\n***INSTALL ROS PYTHON TOOLS***"
apt-get install -y python-ros*




