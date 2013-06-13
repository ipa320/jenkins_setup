#!/bin/bash
export WORKSPACE=$1
$WORKSPACE/setupGraphicDriver.bash
$WORKSPACE/jenkins_setup/scripts/graphicTest/testGlx.bash
