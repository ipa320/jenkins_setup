#!/bin/bash -e

basetgz=${ubuntu_distro}__${arch}__${ros_distro}

sudo rm -rf $WORKSPACE/*

mkdir $WORKSPACE/../aux

scp jenkins@@(SERVERNAME):/home-local/jenkins/chroot_tarballs/$basetgz $WORKSPACE/../aux/
scp jenkins@@(SERVERNAME):/home/jenkins/jenkins-config/.gitconfig $WORKSPACE/.gitconfig
scp -r jenkins@@(SERVERNAME):/home/jenkins/jenkins-config/.ssh $WORKSPACE/.ssh
ls -lah $WORKSPACE

git clone git://github.com/fmw-jk/jenkins_setup.git $WORKSPACE/jenkins_setup
ls -lah $WORKSPACE

cat > $WORKSPACE/env_vars.sh <<DELIM
JOBNAME=$JOB_NAME
ROSDISTRO=$ros_distro
REPOSITORY=$repository
UBUNTUDISTRO=$ubuntu_distro
ARCH=$arch
#TODO
JENKINS_MASTER=@(SERVERNAME)
JENKINS_USER=@(USERNAME)
JOBTYPE=@(JOB_TYPE_NAME)
export ROS_TEST_RESULTS_DIR=/tmp/test_repositories/src_repository/test_results # TODO
DELIM

ls -lah $WORKSPACE

echo "***********ENTER CHROOT************"
echo "*********please be patient*********"
sudo pbuilder execute --basetgz $WORKSPACE/../aux/$basetgz --save-after-exec --bindmounts $WORKSPACE -- $WORKSPACE/jenkins_setup/scripts/pbuilder_env.sh $WORKSPACE

echo "*******CLEANUP WORKSPACE*******"

sudo mv $WORKSPACE/../aux/$basetgz $WORKSPACE/
sudo rm -rf $WORKSPACE/../aux
