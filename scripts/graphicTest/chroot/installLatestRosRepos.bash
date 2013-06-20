#!/bin/bash
mkdir -p ~/ros
# for debugging purposes only
export PATH=/usr/local/bin:$PATH
type git 2>/dev/null || apt-get install -y --force-yes git-core
type pip 2>/dev/null || apt-get install -y --force-yes python-pip
type rosinstall > /dev/null 2>&1 || pip install -U rosinstall

rosinstall ~/ros $DIR/setup.rosinstall
exit 0
