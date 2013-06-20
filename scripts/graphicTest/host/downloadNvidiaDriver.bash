#!/bin/bash
if [ -d /tmp/nvidia ]; then
    echo 'Driver already available'
    exit 0
fi

VERSION=`dpkg -l | grep nvidia-current | sed -rn "s/.*nvidia-current\s*([^ ]+).*/\1/p"`
if [ -z $VERSION ]; then
    echo 'No nvidia-current driver installed'
    exit 1
fi


endpoint=`apt-cache --no-all-versions policy nvidia-current | grep -A1 $VERSION | tail -n 1 | awk '{ print $2 }'`
if [ -z $endpoint ]; then
    echo 'No endpoint found'
    exit 2
fi

filename=`apt-cache --no-all-versions show nvidia-current=$VERSION | grep Filename | awk '{ print $2 }'`
if [ -z $filename ]; then 
    echo 'No filename found'
    exit 3
fi

uri="$endpoint$filename"
uriI386=`echo $uri|sed -r 's/(i386|amd64)/i386/g'`
uriAMD64=`echo $uri|sed -r 's/(i386|amd64)/amd64/g'`

rm -rf /tmp/nvidia
wget $uriI386 -P /tmp/nvidia
wget $uriAMD64 -P /tmp/nvidia
