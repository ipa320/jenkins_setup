#!/bin/bash
export WORKSPACE="/mnt/workspace"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
$DIR/F02InstallAptUtils.bash
$DIR/F03InstallTurboVNC.bash
$DIR/F04InstallVirtualGL.bash
$WORKSAPCE/graphicTest/setupGraphicDriver.bash
