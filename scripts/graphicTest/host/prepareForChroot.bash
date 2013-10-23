#!/bin/bash
export WORKSPACE=$1
export DIR=$WORKSPACE/jenkins_setup/scripts/graphicTest/host

# Download the the exact same version of the nvidia-driver and
# store it in /tmp so we can install it in the chroot. That's
# necessary for glx to work.
# The host system is required to use an nvidia driver at the
# moment.
$DIR/downloadNvidiaDriver.bash
