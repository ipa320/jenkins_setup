#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
$WORKSPACE/graphicTest/setupGraphicDriver.bash
$DIR/testGlx.bash
