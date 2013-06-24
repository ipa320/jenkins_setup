#!/bin/bash
export WORKSPACE=$1
. $WORKSPACE/env_vars.sh
env
export DIR=$WORKSPACE/jenkins_setup/scripts/graphicTest/chroot

. $DIR/remoteX.bash

$DIR/checkDisplayNull.bash &&
$DIR/setupSources.bash &&
$DIR/../tvnc/installTurboVNC.bash &&
$DIR/../vgl/installVirtualGL.bash &&
$DIR/installNvidia.bash &&
$DIR/installLatestRosRepos.bash &&
$DIR/installSimulator.bash &&
$DIR/installTest.bash &&
startX &&
$DIR/startTest.bash;
stopX

$DIR/cleanXmlOutput.bash
