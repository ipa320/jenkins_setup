#!/bin/bash -ex
WORKSPACE=$1
echo $WORKSPACE

env

cd
ls -lah
cd $WORKSPACE 
ls -lah

. ./env_vars.sh

export PATH=$PATH:/usr/local/bin
. /opt/ros/$ROSDISTRO/setup.sh

env

ls -lah

echo "Test"

ls -lah
