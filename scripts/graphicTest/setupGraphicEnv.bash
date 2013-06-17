#!/bin/bash
export WORKSPACE=$1
. $WORKSPACE/env_vars.sh
env
export DIR=$WORKSPACE/jenkins_setup/scripts/graphicTest

$WORKSPACE/setupGraphicDriver.bash
$DIR/installPackages.bash
