#!/bin/bash
export TURBO="/opt/TurboVNC/bin"
export VGL="/opt/VirtualGL/bin"
startX(){
    for i in {1..20}; do
        $TURBO/vncserver :$i -noauth
        if [ $? -eq 0 ]; then break; fi
    done
    export DISPLAY=:$i
}
stopX(){
    if [ -z "$DISPLAY" ]; then
        echo "DISPLAY not set. Cannot stop VNC"
    else
        $TURBO/vncserver -kill $DISPLAY
    fi
}
