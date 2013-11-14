#!/bin/bash
if [ -d /tmp/nvidia ]; then
    echo 'Driver already available'
    exit 0
fi

for driver in "nvidia-304" "nvidia-experimental-304" "nvidia-current"; do
    echo ""
    echo "Searching for '$driver'"
    echo ""

    VERSION=`dpkg -l | grep -E "[ ]+$driver[ ]+" | awk '{ print $3 }'`
    if [ -z "$VERSION" ]; then
        echo "'$driver' not found"
        continue
    fi

    endpoint=`apt-cache --no-all-versions policy $driver | grep -A1 $VERSION | tail -n 1 | awk '{ print $2 }'`
    if [ -z "$endpoint" ]; then
        echo 'No endpoint found'
        exit 2
    fi

    filename=`apt-cache --no-all-versions show $driver=$VERSION | grep Filename | awk '{ print $2 }'`
    if [ -z "$filename" ]; then 
        echo 'No filename found'
        exit 3
    fi

    uri="$endpoint$filename"
    uriI386=`echo $uri|sed -r 's/(i386|amd64)/i386/g'`
    uriAMD64=`echo $uri|sed -r 's/(i386|amd64)/amd64/g'`

    rm -rf /tmp/nvidia
    wget $uriI386 -P /tmp/nvidia
    wget $uriAMD64 -P /tmp/nvidia

    exit 0
done

echo ""
echo "-------------------------------------------------"
echo "Error: Could not find any nvidia-driver installed"
echo "-------------------------------------------------"
echo ""
exit 4
