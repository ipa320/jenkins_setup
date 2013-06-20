#!/bin/bash
apt-get install -y ros-electric-care-o-bot ros-electric-desktop-full ros-electric-simulator-gazebo ros-electric-pr2
apt-get install -y mysql-common libmysqlclient16 libmysqlclient-dev libmysqlclient16-dev
apt-get install -y libwxgtk2.8-0 libwxgtk2.8-dev python-wxgtk2.8 python-wxversion wx2.8-headers

#dpkg -l > $WORKSPACE/dpkg_output.log

