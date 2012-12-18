set -ex
/bin/echo "vvvvvvvvvvvvvvvvvvv  dispatch.sh vvvvvvvvvvvvvvvvvvvvvv"

# pass all arguments to the dispatcher on to the script we're running
SCRIPT_ARGS=""
for i in $*
  do 
  SCRIPT_ARGS=`echo $SCRIPT_ARGS $i`
done
echo "Arguments for script: " $SCRIPT_ARGS

#always ask for pbuilder to make sure we have the updated patched version
#sudo apt-get update  --> this is done in the Jenkins script, right before dispatch is called
sudo apt-get -y install pbuilder

#  get latest version of jenkins scripts
cd $WORKSPACE
if [ -e $WORKSPACE/run_debug_mode ] ; then
  /bin/echo "DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG "
  /bin/echo "DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG "
  /bin/echo "DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG "
  /bin/echo "DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG "
  /bin/echo "DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG "
  /bin/echo "DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG DEBUG  DEBUG "
else
  rm -rf cob_jenkins_setup #cob_jenkins_scripts
  git clone http://github.com/fmw-jk/cob_jenkins_scripts.git #TODO
  cd cob_jenkins.git && git log -n 1
fi

cd $WORKSPACE
export > env
sudo mkdir -p /var/cache/pbuilder/ccache
sudo chmod a+w /var/cache/pbuilder/ccache

cat > pbuilder-env.sh <<EOF
#!/bin/bash -ex
/bin/echo "vvvvvvvvvvvvvvvvvvv  pbuilder-env.sh vvvvvvvvvvvvvvvvvvvvvv"
export CCACHE_DIR="/var/cache/pbuilder/ccache"
export PATH="/usr/lib/ccache:${PATH}"
export WORKSPACE=$WORKSPACE
export OS_PLATFORM=$OS_PLATFORM
export ARCH=$ARCH

if [ -d $WORKSPACE/.ssh ]; then
  cp -a $WORKSPACE/.ssh /root
  chown -R root.root /root/.ssh
fi
if [ -d $WORKSPACE/.subversion ]; then
  cp -a $WORKSPACE/.subversion /root
  chown -R root.root /root/.subversion
fi
cd $WORKSPACE
chmod 755 $WORKSPACE/cob_jenkins_scripts/${SCRIPT}

echo "============================================================"
echo "==== Begin" $SCRIPT "script.    Ignore the output above ====="
echo "============================================================"

$WORKSPACE/cob_jenkins_scripts/${SCRIPT} ${SCRIPT_ARGS}

echo "============================================================"
echo "==== End" $SCRIPT "script.    Ignore the output below ====="
echo "============================================================"

EOF

chmod 755 pbuilder-env.sh

#TOP=$(cd `dirname $0` ; /bin/pwd) #TODO what for??

#/usr/bin/env

sudo pbuilder execute \
    --basetgz $basetgz \
    --bindmounts "/var/cache/pbuilder/ccache $WORKSPACE" \
    --inputfile $WORKSPACE/cob_jenkins_scripts/$SCRIPT \
    -- $WORKSPACE/pbuilder-env.sh $SCRIPT


/bin/echo "^^^^^^^^^^^^^^^^^^  dispatch.sh ^^^^^^^^^^^^^^^^^^^^"
