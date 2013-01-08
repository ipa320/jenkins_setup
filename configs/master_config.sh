#!/bin/bash

echo "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
echo "This script will help you set up the configuration of the master."
echo "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
echo ""

echo "Setting up master configuration"
if [ -d ~/jenkins-config ]; then
    read -p "'jenkins-config'-folder already exists! Do you want to update the current configurations? [y|n] " answ
    case "$answ" in
        Yes|yes|Y|y|"") echo "Updating 'jenkins-config'-folder!"
            ;;
        No|no|N|n) echo "'jenkins-config'-folder will not be updated!"
            read -p "Do you want to execute 'ssh-copy-id' for each entry in 'slavelist'? [y|n] " resp
            case "$resp" in 
                Yes|yes|Y|y|"")
                    echo "Installing public key of master on each entry in 'slavelist' via ssh-copy-id to enable password-free communication"
                    while read line; do
                        echo "  - $line"
                        ssh-copy-id jenkins@"$line"
                    done <~/jenkins-config/slavelist
                    ;;
                *)
                    echo "Exiting without changing anything"
                    ;;
            esac
            exit 0
            ;;
        *) echo "Unknown parameter"
            exit 1
            ;;
    esac
else
    echo "Creating 'jenkins-config'-folder"
    mkdir ~/jenkins-config
fi

update=False
if [ -f ~/jenkins-config/.gitconfig ]; then
    read -p "'.gitconfig' already exists! Do you want it to update? [y|n] " resp
    case "$resp" in
        Yes|yes|Y|y|"") update=True
            ;;
        No|no|N|n)  echo "'.gitconfig' will not be updated"
            ;;
        *) echo "Unknown parameter! '.gitconfig' will not be updated"
            ;;
    esac
fi
if [ ! -f ~/jenkins-config/.gitconfig ] || [ $update == True ]; then
    read -p "Enter github login: " github_login
    read -p "Enter github password: " github_pw
    #TODO more entries?
    cat >~/jenkins-config/.gitconfig <<DELIM
[github]
        user = $github_login
        password = $github_pw
DELIM
fi

update=False
if [ -d ~/jenkins-config/.ssh ]; then
    read -p "'.ssh/' already exists! Do you want it to update? [y|n] " resp
    case "$resp" in
        Yes|yes|Y|y|"") update=True
            ;;
        No|no|N|n)  echo "'.ssh/' will not be updated"
            ;;
        *) echo "Unknown parameter! '.ssh/' will not be updated"
            ;;
    esac
fi
if [ ! -d ~/jenkins-config/.ssh ] || [ $update == True ]; then
    echo "Copying ~/.ssh to 'jenkins-config'-folder"
    cp -R ~/.ssh ~/jenkins-config/
fi

if [ -f ~/jenkins-config/slavelist ]; then
    echo "'slavelist' already exists! If you want to change or reset it, please do it manually."
    echo "Installing public key of master on each entry in 'slavelist' via ssh-copy-id to enable password-free communication"
    while read line; do
        echo "  - $line"
        ssh-copy-id jenkins@"$line"
    done <~/jenkins-config/slavelist
else
    touch ~/jenkins-config/slavelist
    echo "'slavelist' did not exist and was created."
fi
