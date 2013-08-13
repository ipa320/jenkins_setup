#!/bin/bash -e
echo "vvvvvvvvvvvvvvvvvvv  pbuilder_env.sh vvvvvvvvvvvvvvvvvvvvvv"
date
export WORKSPACE=$1
echo $WORKSPACE

echo "Set up environment variables"
. $WORKSPACE/env_vars.sh

export PATH=$PATH:/usr/local/bin
. /opt/ros/$ROSDISTRO/setup.sh
export ROS_PACKAGE_PATH=/tmp/test_repositories/src_repository:$ROS_PACKAGE_PATH
export PYTHONPATH=$WORKSPACE/jenkins_setup/src:$PYTHONPATH

env

echo "Set up ssh"
cp -a $WORKSPACE/.ssh /root
ls -la /root/
chown -R root.root /root/.ssh

#echo "Install python-catkin-pkg python-rosdistro rosinstall"
#apt-get update
#apt-get upgrade -y
#apt-get install python-catkin-pkg python-rosdistro -y
#apt-get install openssh-client -y

#pip install -U rosinstall

echo "============================================================"
echo "==== Begin" $SCRIPT "script.    Ignore the output above ===="
echo "============================================================"

date
$WORKSPACE/jenkins_setup/scripts/${JOBTYPE}.py $PIPELINE_REPOS_OWNER $JENKINS_MASTER $JENKINS_USER $ROSDISTRO $REPOSITORY
date
echo "============================================================"
echo "==== End" $SCRIPT "script.    Ignore the output below ======"
echo "============================================================"


#echo "============================================================"
#echo "DEBUG"
#rospack list

#echo "============================================================"
#rosstack list

#apt-get install tree
#tree $WORKSPACE

#tree /tmp/test_repositories
