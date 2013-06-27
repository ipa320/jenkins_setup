#!/bin/bash
if [ ! -d ~/.ssh ] && [ ! -d /root/.ssh ]; then
    echo 'No .ssh keys in ~/.ssh or /root/.ssh found'
    exit 1
fi
if [ -d ~/.ssh ] && [ ! -d /root/.ssh ]; then
    cp -r ~/.ssh /root/
fi
if [ -d /root/.ssh ] && [ ! -d ~/.ssh ]; then
    cp -r /root/.ssh ~/
fi
