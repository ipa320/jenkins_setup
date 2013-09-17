#!/bin/bash
distro=$1
if [ -z "$distro" ]; then
    echo "Usage: ./installSimulator.bash [ROS-distro]"
    exit 1
fi
apt-get install -y --force-yes ros-$distro-care-o-bot ros-$distro-desktop-full ros-$distro-simulator-gazebo ros-$distro-pr2
if [ $? != 0 ]; then
    echo ''
    echo '----------------------------------------------'
    echo 'Error occured during installation of simulator'
    echo '----------------------------------------------'
    echo ''
    exit 2
fi

apt-get install -y --force-yes mysql-common libmysqlclient16 libmysqlclient-dev libmysqlclient16-dev
if [ $? != 0 ]; then
    echo ''
    echo '----------------------------------------------'
    echo 'Warning: Could not install important packages'
    echo '----------------------------------------------'
    echo ''
    exit 3
fi

apt-get install -y --force-yes libwxgtk2.8-0 libwxgtk2.8-dev python-wxgtk2.8 python-wxversion wx2.8-headers
if [ $? != 0 ]; then
    echo ''
    echo '----------------------------------------------'
    echo 'Warning: Could not install important packages'
    echo '----------------------------------------------'
    echo ''
    exit 4
fi

exit 0
#dpkg -l > $WORKSPACE/dpkg_output.log

