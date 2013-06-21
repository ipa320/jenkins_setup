#!/bin/bash
type xhost  2>/dev/null || sudo apt-get install xhost
xhost + 
sudo rm -rf /tmp/.Xauthority
cp ~/.Xauthority /tmp
