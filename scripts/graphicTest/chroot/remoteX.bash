#!/bin/bash
export TURBO="/opt/TurboVNC/bin"
export VGL="/opt/VirtualGL/bin"
startX(){
    for i in $(seq 1 20); do
        if [ -e "/tmp/.X$i-lock" ]; then
            echo "DISPLAY :$i already used. Trying next one"
        else
            echo "Starting display on $i"
            $TURBO/vncserver :$i -noauth
            break;
        fi
    done
    export DISPLAY=:$i
    echo "VNC okay. New DISPLAY is $DISPLAY"
}
stopX(){
    if [ -z "$DISPLAY" ]; then
        echo "DISPLAY not set. Cannot stop VNC"
    else
        $TURBO/vncserver -kill $DISPLAY
    fi
}
