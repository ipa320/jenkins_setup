#!/bin/bash -ex
export WORKSPACE=$1
echo $WORKSPACE

env

. $WORKSPACE/env_vars.sh

export PATH=$PATH:/usr/local/bin
. /opt/ros/$ROSDISTRO/setup.sh
export ROS_PACKAGE_PATH=/tmp/test_repositories/src_repository:$ROS_PACKAGE_PATH
export PYTHONPATH=$WORKSPACE/jenkins_setup/src:$PYTHONPATH

env

cp $WORKSPACE/.gitconfig ~/.gitconfig
cp -a $WORKSPACE/.ssh /root
chown -R root.root /root/.ssh

ls -lah

echo "Test"

cd

ls -lah

cd $WORKSPACE

ls -lah
