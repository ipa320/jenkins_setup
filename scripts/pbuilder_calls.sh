#!/bin/bash

case $1 in
    create)
        # parameter: basetgz, ubuntu distro, arch
        echo "Going to create $2 with arguments $3 and $4"
        sudo pbuilder --create --basetgz $2 --distribution $3 --architecture $4 \
            --debootstrapopts --variant=buildd --components "main universe multiverse" \
            --debootstrapopts --keyring=/etc/apt/trusted.gpg
        ;;
    update)
        # parameter: basetgz
        echo "Going to update $2"
        sudo pbuilder --update --basetgz $2
        ;;
    execute)
        # parameter: basetgz, script name, script arguments
        cnt=1
        SCRIPT_ARGS=""
        for i in $*; do
            if [ $cnt -gt 3 ] ; then
                SCRIPT_ARGS=`echo $SCRIPT_ARGS $i`
            fi
            cnt=$(($cnt+1))
        done
        echo "Going to execute $3 in $2 with the arguments $SCRIPT_ARGS"
        sudo pbuilder --execute --basetgz $2 --save-after-exec -- $3 $SCRIPT_ARGS

esac
