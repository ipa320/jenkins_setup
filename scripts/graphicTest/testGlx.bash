#!/bin/bash
TURBO="/opt/TurboVNC/bin"
VGL="/opt/VirtualGL/bin"
#apt-get install -y timeout
for i in {1..20}; do
    $TURBO/vncserver :$i
    if [ $? -eq 0 ]; then break; fi
done
export DISPLAY=:$i
timeout -9 10 $VGL/vglrun $VGL/glxspheres64
vncserver -kill :$i
