#!/bin/bash
apt-get install -y --force-yes timeout
export DISPLAY=:0
timeout -9 10 $DIR/glxspheres64
