#!/bin/bash
export WORKSPACE=$1
export DIR=$WORKSPACE/jenkins_setup/scripts/graphicTest/host

type xhost 2>/dev/null || { echo 'You need xhost installed'; exit 1; }
xhost +
$DIR/downloadNvidiaDriver.bash
