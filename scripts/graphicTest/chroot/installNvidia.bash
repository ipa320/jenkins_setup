#!/bin/bash
apt-get install -y --force-yes --reinstall libgl1-mesa-glx
apt-get install -y --force-yes nvidia-current
if [ $? != 0 ]; then
    echo ''
    echo '---------------------------------------------------------'
    echo 'Could not install the latest nvidia-driver nvidia-current'
    echo '---------------------------------------------------------'
    echo ''
    exit 1
fi

rm -rf tmpNvidiaFiles; mkdir tmpNvidiaFiles; cd tmpNvidiaFiles
arch=`dpkg --print-architecture`
file=`ls /tmp/nvidia | grep $arch`
if [ -z "$file" ]; then
	echo "Cannot find appropriate nvidia-driver in /tmp/nvidia"
	exit 2
fi

# extract patching driver deb-content to current directory
# ---------------------------------------------------------------------
ar -x /tmp/nvidia/$file


# copy content of data.tar.gz to / and run ldconfig
# ---------------------------------------------------------------------
tar -xzvf data.tar.gz -C /
ldconfig


# update postinst script in control.tar.gz
# ---------------------------------------------------------------------
tar -xzvf control.tar.gz
if [ ! -f postinst ] || [ ! -f control ]; then
    echo ""
    echo "----------------------------------------------------"
    echo "Invalid control.tar.gz. No postinst or control found"
    echo "----------------------------------------------------"
    echo ""
    exit 3
fi
CVERSION=`cat control | grep "^Version" | awk '{print $2}' | awk -F "-" '{print $1}' | cut -d\: -f2`
if [ -z "$CVERSION" ]; then
    echo ""
    echo "---------------------------------------------"
    echo "Could not determine patching driver's version"
    echo "---------------------------------------------"
    echo ""
    exit 4
fi
if [ -z "`cat postinst | sed -n '8s/CVERSION//p'`" ]; then
    echo ""
    echo "------------------------------------------"
    echo "Unknown postinst format. No CVERSION found"
    echo "------------------------------------------"
    echo ""
    exit 5
fi
cat postinst | sed "8s/^.*$/CVERSION=\"$CVERSION\"/" > postinst2
chmod a+x postinst2
./postinst2 configure

# remove artifacts
# ---------------------------------------------------------------------
cd ..; rm -rf tmpNvidiaFiles
exit 0
