#!/bin/bash
export WORKSPACE=$1
export DIR=$WORKSPACE/jenkins_setup/scripts/graphicTest
$WORKSPACE/setupGraphicDriver.bash
$DIR/installPackages.bash
