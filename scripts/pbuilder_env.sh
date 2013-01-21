#!/bin/bash -ex
export WORKSPACE=$1
echo $WORKSPACE

. $WORKSPACE/env_vars.sh

export PATH=$PATH:/usr/local/bin
. /opt/ros/$ROSDISTRO/setup.sh
export ROS_PACKAGE_PATH=/tmp/test_repositories/src_repository:$ROS_PACKAGE_PATH
export PYTHONPATH=$WORKSPACE/jenkins_setup/src:$PYTHONPATH

cp $WORKSPACE/.gitconfig ~/.gitconfig
cp -a $WORKSPACE/.ssh /root
chown -R root.root /root/.ssh

apt-get update
apt-get upgrade -y
apt-get install python-catkin-pkg python-rosdistro -y

pip install -U rosinstall

$WORKSPACE/jenkins_setup/scripts/build.py $JENKINS_MASTER $JENKINS_USER $ROSDISTRO $REPOSITORY
