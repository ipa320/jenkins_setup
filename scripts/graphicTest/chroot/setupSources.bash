#!/bin/bash
pattern="s/^\s*deb (http:[^ ]+)\s+([^ ]+)(.*)/"
url=`cat /etc/apt/sources.list | sed -rn "$pattern\1/p"`
dist=`cat /etc/apt/sources.list | sed -rn "$pattern\2/p"`
pools=`cat /etc/apt/sources.list | sed -rn "$pattern\3/p"`

sudo cp /etc/apt/sources.list /etc/apt/sources.list.bkp
sudo cat > /etc/apt/sources.list << EOF
deb $url $dist restricted $pools
deb $url $dist-updates restricted $pools
EOF

sudo apt-get update
