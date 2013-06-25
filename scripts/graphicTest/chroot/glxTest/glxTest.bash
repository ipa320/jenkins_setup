#!/bin/bash
VGL="/opt/VirtualGL/bin/vglrun"
type timeout 1>/dev/null 2>&1 || apt-get install -y --force-yes timeout
CMD="$VGL $DIR/glxTest/glxspheres64"
timeout -9 10 $CMD || timeout -s 9 10 $CMD
