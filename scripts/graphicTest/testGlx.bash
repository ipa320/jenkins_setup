#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
apt-get install -y timeout
export DISPLAY=:0
timeout -9 10 $DIR/glxspheres64
