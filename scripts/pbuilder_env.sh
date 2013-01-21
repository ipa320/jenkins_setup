#!/bin/bash -ex
env

cd 

. ./env_vars.sh

export PATH=$PATH:/usr/local/bin
. /opt/ros/$ROSDISTRO/setup.sh

env

ls -lah

echo "Test"

ls -lah
