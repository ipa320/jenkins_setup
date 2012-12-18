#!/usr/bin/env bash
if [ $# -lt 4 ] ;  then
  echo "Usage:" $0 "ubuntu_distro arch workspace script [script_args]"
  exit
fi

# support relative workspace
export OS_NAME="ubuntu"
export IMAGETYPE="cob_jenkins_tools"
export IMAGEVERSION="0.1"

# get arguments of run_local
export UBUNTU_DISTRO=$1
export OS_PLATFORM=$1
export ARCH=$2
export WORKSPACE=$(readlink -f $3)
export SCRIPT=$4

# get extra arguments of script
cnt=1
SCRIPT_ARGS=""
for i in $*
do
  if [ $cnt -gt 4 ] ; then
    SCRIPT_ARGS=`echo $SCRIPT_ARGS $i`
  fi
  cnt=$(($cnt+1))
done

# print arguments
echo "  - ubuntu distro: " $UBUNTU_DISTRO
echo "  - architecture:  " $ARCH
echo "  - script:        " $SCRIPT
echo "  - workspace:     " $WORKSPACE
echo "  - script args:   " $SCRIPT_ARGS

# dispatch the chroot
cd $WORKSPACE
bash cob_chroot_dispatch.sh $SCRIPT_ARGS
