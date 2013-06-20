#!/bin/bash
apt-get install -y nvidia-current
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
if [ -z $file ]; then
	echo "Cannot find appropriate nvidia-driver in /tmp/nvidia"
	exit 1
fi

ar -x /tmp/nvidia/$file
tar -xzvf data.tar.gz -C /
ldconfig
cd ..; rm -rf tmpNvidiaFiles
