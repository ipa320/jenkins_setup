# Jenkins Guide

This repository contains the code (config, src and script files) to set up and run a Cob-Jenkins CI Server using the Cob-Pipeline-PlugIn.

## Prerequisites and assumptions
Before starting with this guide, please setup one machine with the following properties:
- Operation system: Ubuntu 12.04
- user: jenkins (admin)

assumptions:
- we're only using one machine which is master and slave at the same time
- apt-cacher is running on master
- there's a github user that has read access to all repositories which should be build and write access to a jenkins_config repository (e.g. http://github.com/ipa320/jenkins_config)


## Jenkins installation

### Debian packages for Ubuntu
Install basic packages

    sudo apt-get install git-core pbuilder devscripts pigz
    
Install basic ROS packages

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update && sudo apt-get install ros-groovy-ros


Add the jenkins debian repository and install jenkins

    wget -q -O - http://pkg.jenkins-ci.org/debian/jenkins-ci.org.key | sudo apt-key add -
    sudo su -c 'echo "deb http://pkg.jenkins-ci.org/debian binary/" > /etc/apt/sources.list.d/jenkins.list'
    sudo apt-get update && sudo apt-get install jenkins

### Up or downgrade jenkins to version v1.514
We've tested the setup on Jenkins version v1.514. You can find the war file [here](http://mirrors.jenkins-ci.org/war).

    cd /usr/share/jenkins/
    sudo rm -rf jenkins.war
    sudo wget http://mirrors.jenkins-ci.org/war/1.514/jenkins.war

restart jenkins

    sudo /etc/init.d/jenkins restart


After a successfull installation you can access the jenkins server in your browser at [http://localhost:8080](http://localhost:8080).


## Jenkins configuration

### Global security
Go to [http://localhost:8080/configureSecurity](http://localhost:8080/configureSecurity)

- Check *Jenkins's own user database* under *Access Control*/*Security Realm*. And check **Allow users to sign up**.
- Set *Authorization* to **Project-based Matrix Authorization Strategy**.
- Add an `admin`-user and give him all rights.
- Add an `anonymous`-group and an `authenticated`-group and give them rights according to the screentshot.

After click save the Server will throw you to a Login screen. Just register with the username `admin`.

![Project-based Matrix Authorization Strategy](./authentication.png "Example for Project-based Matrix Authorization Strategy")

### Basic configuration
Go to [http://localhost:8080/configure](http://localhost:8080/configure)

- Set *# of executors* to `1`.
- Set *Jenkins URL* to your servers name.

You can keep the default values for all other entries.


### Jenkins plugin installation
Go to [http://localhost:8080/pluginManager/available](http://localhost:8080/pluginManager/available) and install the following plugins:

- Parameterized Trigger Plugin https://wiki.jenkins-ci.org/display/JENKINS/Parameterized+Trigger+Plugin
- Build Pipeline Plugin http://code.google.com/p/build-pipeline-plugin/
- Mailer https://wiki.jenkins-ci.org/display/JENKINS/Mailer
- View Job Filters https://wiki.jenkins-ci.org/display/JENKINS/View+Job+Filters

### Install `jenkins_setup`, `jeknins_config` and the *cob-pipeline* plugin
Download the *.hpi* file from https://github.com/fmw-jk/cob-pipeline-plugin/releases and place it in `/var/lib/jenkins/plugins`.

    cd /var/lib/jenkins/plugins
    sudo wget https://github.com/fmw-jk/cob-pipeline-plugin/releases/download/v0.9.5-alpha/cob-pipeline.hpi

All scripts and configurations will be stored in `/home/jenkins/jenkins-config`.

    mkdir ~/jenkins-config

All tarballs will be stored in ~/chroot_tarballs (adapt the *JENKINS_MASTER_NAME*)

    mkdir -p ~/chroot_tarballs
    mkdir -p ~/chroot_tarballs/in_use_on__<JENKINS_MASTER_NAME>

Setup ssh configuration

    ssh-keygen -f ~/jenkins-config
    ssh-keyscan -H github.com > ~/jenkins-config

You have to add this key to your GitHub user http://github.com/settings/ssh. 

     cat ~/jenkins-config/.ssh/id_rsa.pub

Setup git configuration on master

    git config --global user.name "<USER_NAME>"
    git config --global user.email "<EMAIL>"

Clone the `jenkins_setup` and `jenkins_config` repositories

    git clone git@github.com:ipa320/jenkins_config.git ~/jenkins-config/jenkins_config
    git clone git@github.com:ipa320/jenkins_setup.git ~/jenkins-config/jenkins_setup

Add the `jenkins_setup` module to the `$PYTHONPATH` (adapt the *ROS_RELEASE*).

    echo "export PYTHONPATH=~/jenkins-config/jenkins_setup/src" > /etc/profile.d/python_path.sh
    echo "source /opt/ros/<ROS_RELEASE>/setup.sh" >> /etc/profile.d/python_path.sh

Enable passwordless sudo rights for the jenkins user by adding the following line at the end of `/etc/sudoers` (open with `sudo visudo -f /etc/sudoers`).

    jenkins    ALL=(ALL) NOPASSWD: ALL

Enable password-less ssh login from master to slave and slave to master.

    ssh-copy-id <master>    # _on slave_
    ssh <master>            # _on slave_
    ssh-copy-id <slave>     # _on master_

Afterwards reboot the Jenkins-Server

    sudo reboot now

#### Performance improvements (optional)
Using RAM for chroot environment and parallel compression.

Add the following line to `/etc/fstab`

    # pbuilder
    tmpfs   /var/cache/pbuilder/build   tmpfs   defaults,size=32000M    0   0

Mount *tmpfs* by entering

    sudo mount -a

Create a file called `~/.pbuilderrc`

    touch ~/.pbuilderrc

Add the following content to `~/.pbuilderrc`

    # don't use aptcache
    APTCACHE=""

    # ccache
    sudo mkdir -p /var/cache/pbuilder/ccache
    sudo chmod a+w /var/cache/pbuilder/ccache
    export CCACHE_DIR="/var/cache/pbuilder/ccache"
    export PATH="/usr/lib/ccache:${PATH}"
    EXTRAPACKAGES=ccache
    BINDMOUNTS="${CCACHE_DIR}"

    # pigz; multicore zipping
    COMPRESSPROG=pigz
    
    # tmpfs
    APTCACHEHARDLINK=no

Afterwards reboot the Jenkins-Server

    sudo reboot now

### Configure the *cob-pipeline* plugin
Go to the *cob pipeline configuration* section at [http://fmw-xps:8080/configure[(http://fmw-xps:8080/configure) and fill the following fields (As soon as you fill out the fields, the values will be validated in the background.):

- Jenkins Admin Login/Password (This is the user you configured before in the Configure Security part with all the permissions. Enter its login name and password.)
- Configuration Folder (Enter the path of the cob-pipeline configuration folder.)

```
    /home/jenkins/jenkins-config
```

- Tarball Location (enter the location where the tarballs are stored.)

```
    jenkins@fmw-xps:/home/jenkins/chroot_tarballs
```

- GitHub User Login/Password (This is the github user that has read-permission to all the repositories you want to be tested. It has also write-permission to your jenkins_config repository.)
- Pipeline Repositories Owner/Fork (GitHub user that ownes the *jenkins_setup* and the *jenkins_config* repository.)

```
    ipa320
```

- ROS releases (ROS distributions that should be supported by your build/test pipeline)

```
    groovy hydro
```

- Robots (Nodes which can be chosen for Hardware Build/Test jobs.)

keep empty if you have no hardware slaves

- Target Platform Url (URL where the ROS `targets.yaml` is stored, defining the Ubuntu target platforms for each ROS Version.)

```
    https://raw.github.com/ipa320/jenkins_setup/master/releases/targets.yaml
```

### configure default view
*TODO!!!*

### configure mailer
*TODO!!!*
