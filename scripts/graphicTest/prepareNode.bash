#!/bin/bash
export DIR="$(cd $(dirname $0); pwd -P)"

echo '---------------------------------'
echo '       Install TurboVNC'
echo '---------------------------------'
echo
$DIR/tvnc/installTurboVNC.bash

echo
echo
echo '---------------------------------'
echo '       Install VirtualGL'
echo '---------------------------------'
echo
$DIR/vgl/installVirtualGL.bash
