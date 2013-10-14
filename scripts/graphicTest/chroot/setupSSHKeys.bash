#!/bin/bash
if [ ! -d $WORKSPACE/.ssh ]; then
    echo 'No .ssh keys in $WORKSPACE/.ssh found'
    exit 1
fi
cp -r $WORKSPACE/.ssh ~/
cp -r $WORKSPACE/.ssh /root/
chmod 700 ~/.ssh
chmod 700 /root/.ssh

user=`whoami`
chown -R $user:$user ~/.ssh
chown -R $user:$user /root/.ssh
