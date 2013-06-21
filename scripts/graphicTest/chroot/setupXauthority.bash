#!/bin/bash

if [ ! -f /tmp/.Xauthority ];
    echo 'Could not find .Xauthority file in /tmp'
    exit 1
fi

mkdir -p ~/
mv /tmp/.Xauthority ~
chown root:root ~/.Xauthority
chmod 600 ~/.Xauthority
