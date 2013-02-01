#!/bin/bash

echo "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
echo "This script will help you set up the configuration of the slave."
echo "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
echo ""

read -p "Please enter name or IP of master: " master
ping -c 5 -W 1 -i .2 -q "$master"
case "$?" in
    0) echo "Found master"
        ;;
    1) echo "Master did not answer within timeout"
        exit 1
        ;;
    2) echo "Unknown host"
        exit 1
        ;;
    *) echo "ERROR: unknown return code"
        exit 1
        ;;
esac

#TODO check whether ssh is setup
echo "Installing slaves public key on master via ssh-copy-id to enable password-free communication"
#TODO check if jenkins user exists
ssh-copy-id jenkins@"$master"

if ssh jenkins@"$master" 'ls ~/jenkins-config/slavelist >/dev/null'; then
    echo "Given host is configured as Jenkins master"
else
    echo "Given host is not configured as Jenkins master yet. Please run 'master_config.sh' on master first!"
    exit 1
fi

echo ""
known=false
scp jenkins@"$master":~/jenkins-config/slavelist ~/jenkins-config/slavelist
while read line; do
    if [ $line == $HOSTNAME ]; then
        echo "$HOSTNAME already known as slave"
        known=true
    fi
done <~/jenkins-config/slavelist

echo $known
if [ $known==false ]; then
    echo "Adding $HOSTNAME to slavelist of $master"
    echo $HOSTNAME >>~/jenkins-config/slavelist
    scp ~/jenkins-config/slavelist jenkins@"$master":~/jenkins-config/slavelist
fi
rm ~/jenkins-config/slavelist

echo ""
echo "Setting up slave configuration"
if [ -d ~/jenkins-config ]; then
    read -p "'jenkins-config'-folder already exists! Do you want to overwrite current configurations? [y|n] " answ
    case "$answ" in
        Yes|yes|Y|y|"") echo "Deleting 'jenkins-config'-folder!"
                        rm -rf ~/jenkins-config
            ;;
        No|no|N|n)  echo "Aborting."
                    exit 0
            ;;
        *) echo "Unknown parameter"
            exit 1
            ;;
    esac
fi

echo "Creating 'jenkins-config'-folder"
mkdir ~/jenkins-config

cd ~/jenkins-config
scp jenkins@"$master":~/jenkins-config/slave_config.yaml .

echo "Configuration file copied from Jenkins master"
