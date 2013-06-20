#!/bin/bash
mkdir -p ~/ros
# for debugging purposes only
apt-get -y install python-pip git-core
export PATH=/usr/local/bin:$PATH
type rosinstall > /dev/null 2>&1 || pip install -U rosinstall

rosinstall ~/ros $DIR/setup.rosinstall
