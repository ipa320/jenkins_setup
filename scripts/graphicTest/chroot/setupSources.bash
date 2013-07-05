#!/bin/bash
if [ ! -z "`cat /etc/apt/sources.list | grep '\-updates restricted'`" ]; then
    echo 'Sources already set up'
    exit 0
fi

pattern="s/^\s*deb (http:[^ ]+)\s+([^ ]+)(.*)/"
url=`cat /etc/apt/sources.list | sed -rn "$pattern\1/p"`
dist=`cat /etc/apt/sources.list | sed -rn "$pattern\2/p"`
pools=`cat /etc/apt/sources.list | sed -rn "$pattern\3/p"`
if [ -z "$url" ] || [ -z "$dist" ] || [ -z "$pools" ]; then
    echo ''
    echo '-------------------------------------'
    echo 'Could not parse /etc/apt/sources.list'
    echo '-------------------------------------'
    echo ''
    exit 1
fi

sudo cp /etc/apt/sources.list /etc/apt/sources.list.bkp
sudo cat > /etc/apt/sources.list << EOF
deb $url $dist restricted $pools
deb $url $dist-updates restricted $pools
EOF

sudo apt-get update
