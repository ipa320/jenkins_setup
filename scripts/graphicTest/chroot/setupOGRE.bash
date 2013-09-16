#!/bin/bash
extraline="export OGRE_RESOURCE_PATH=/usr/lib/OGRE:\$OGRE_RESOURCE_PATH"
files=`find /opt/ros -name "setup.*sh" -ipath "*simulator_gazebo*"`
echo $files

for file in $files; do
    status="SKIPPED"
    if [ -z "`cat $file | grep "$extraline"`" ]; then
        echo $extraline >> $file
	status="OKAY"
    fi
    echo "Setting up '$file'... $status"
done

exit 0
