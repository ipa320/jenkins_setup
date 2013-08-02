#!/bin/bash -e
echo "vvvvvvvvvvvvvvvvvvv  pbuilder_env.sh vvvvvvvvvvvvvvvvvvvvvv"
date
export WORKSPACE=$1
echo $WORKSPACE
echo "Job-Type: $JOBTYPE"

echo "Set up environment variables"
. $WORKSPACE/env_vars.sh

export PATH=$PATH:/usr/local/bin
. /opt/ros/$ROSDISTRO/setup.sh
export ROS_PACKAGE_PATH=/tmp/test_repositories/src_repository:$ROS_PACKAGE_PATH
export PYTHONPATH=$WORKSPACE/jenkins_setup/src:$PYTHONPATH



echo "Set up git and ssh"
cp $WORKSPACE/.gitconfig ~/.gitconfig
cp -a $WORKSPACE/.ssh /root
ls -la /root/
chown -R root.root /root/.ssh
case $JOBTYPE in
    graphic_test)
        echo "Set up graphic"
        export DIR=$WORKSPACE/jenkins_setup/scripts/graphicTest/chroot

        . $DIR/remoteX.bash

        $DIR/checkDisplayNull.bash &&
        $DIR/setupSources.bash &&
        $DIR/../tvnc/installTurboVNC.bash &&
        $DIR/../vgl/installVirtualGL.bash &&
        $DIR/installNvidia.bash &&
        startX
        ;;
esac

echo "======================================================"
echo " Listing environment variables for debugging purposes "
echo "======================================================"
env

echo 
echo
echo "============================================================"
echo "==== Begin" $SCRIPT "script.    Ignore the output above ===="
echo "============================================================"

date
if [ $JOBTYPE == "graphic_test" ]; then
    $WORKSPACE/jenkins_setup/scripts/${JOBTYPE}.py $PIPELINE_REPOS_OWNER $JENKINS_MASTER $JENKINS_USER $ROSDISTRO $REPOSITORY true
    stopX
else
    $WORKSPACE/jenkins_setup/scripts/${JOBTYPE}.py $PIPELINE_REPOS_OWNER $JENKINS_MASTER $JENKINS_USER $ROSDISTRO $REPOSITORY false
fi
date
echo "============================================================"
echo "==== End" $SCRIPT "script.    Ignore the output below ======"
echo "============================================================"
