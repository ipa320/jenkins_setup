#!/bin/bash
if [ ! -d /opt/VirtualGL ]; then
    $DIR/../vgl/installVirtualGL.bash
fi
sudo /opt/VirtualGL/bin/vglserver_config -config +s +f -t
