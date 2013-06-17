#!/bin/bash
export WORKSPACE=$1
. $WORKSPACE/env_vars.sh
export DIR=$WORKSPACE/jenkins_setup/scripts/graphicTest
env
exit 1

$WORKSPACE/setupGraphicDriver.bash
$DIR/installPackages.bash
