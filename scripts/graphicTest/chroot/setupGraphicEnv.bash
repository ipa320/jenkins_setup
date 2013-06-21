#!/bin/bash
export WORKSPACE=$1
. $WORKSPACE/env_vars.sh
env
export DIR=$WORKSPACE/jenkins_setup/scripts/graphicTest/chroot

$DIR/checkDisplayNull.bash &&
$DIR/setupSources.bash &&
$DIR/installNvidia.bash &&
$DIR/installLatestRosRepos.bash &&
$DIR/installSimulator.bash &&
$DIR/installTest.bash &&
$DIR/startTest.bash

$DIR/cleanXmlOutput.bash
