#!/bin/bash
export TURBO="/opt/TurboVNC/bin"
export VGL="/opt/VirtualGL/bin"
startX(){
    for i in {1..20}; do
        $TURBO/vncserver :$i
        if [ $? -eq 0 ]; then break; fi
    done
    export DISPLAY=:$i
}
stopX(){
    $TURBO/vncserver -kill $DISPLAY
}
