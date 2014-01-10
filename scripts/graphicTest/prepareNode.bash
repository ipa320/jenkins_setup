#!/bin/bash
#/chroot required since this is the path used by pbuilder_env.sh
export DIR="$(cd $(dirname $0); pwd -P)/chroot"

echo '---------------------------------'
echo '       Install TurboVNC'
echo '---------------------------------'
echo
$DIR/../tvnc/installTurboVNC.bash

echo
echo
echo '---------------------------------'
echo '       Install VirtualGL'
echo '---------------------------------'
echo
$DIR/../vgl/installVirtualGL.bash
