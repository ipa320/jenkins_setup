#!/bin/bash
export WORKSPACE=$1
DIR="$( cd "$( dirname "$0" )" && pwd )" 
$WORKSPACE/setupGraphicDriver.bash
$DIR/testGlx.bash
