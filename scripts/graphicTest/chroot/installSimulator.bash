#!/bin/bash
apt-get install -y --force-yes ros-electric-care-o-bot ros-electric-desktop-full ros-electric-simulator-gazebo ros-electric-pr2
if [ $? != 0 ]; then
    echo ''
    echo '----------------------------------------------'
    echo 'Error occured during installation of simulator'
    echo '----------------------------------------------'
    echo ''
    exit 1
fi

apt-get install -y --force-yes mysql-common libmysqlclient16 libmysqlclient-dev libmysqlclient16-dev
if [ $? != 0 ]; then
    echo ''
    echo '----------------------------------------------'
    echo 'Warning: Could not install important packages'
    echo '----------------------------------------------'
    echo ''
fi

apt-get install -y --force-yes libwxgtk2.8-0 libwxgtk2.8-dev python-wxgtk2.8 python-wxversion wx2.8-headers
if [ $? != 0 ]; then
    echo ''
    echo '----------------------------------------------'
    echo 'Warning: Could not install important packages'
    echo '----------------------------------------------'
    echo ''
fi

exit 0
#dpkg -l > $WORKSPACE/dpkg_output.log

