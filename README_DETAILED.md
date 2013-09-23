# Jenkins Guide (DETAILED)

This repository contains the code (config, src and script files) to set up and run a Cob-[Jenkins CI server](http://jenkins-ci.org) using the [Cob-Pipeline-Plugin](http://github.com/fmw-jk/cob-pipeline-plugin).

This guide is designed for Cob-Pipeline developers, those who want to setup a efficient test framework and those who just want to know more about it.
**If you want to set up the Cob-Pipeline quickly on one computer and only use it, the [minimal Jenkins Guide](README.md) is what you are looking for.**
Below you will find a detailed description of the purposes of the Cob-Pipeline and the setup for **one master** and **multiple slave** nodes.
Nevertheless read first the short description of the setup process in the [minimal Jenkins Guide](README.md).
Further information are given below.

###Version
The plugin and this manual are designed and tested for Jenkins CI v1.514.

###Table of Contents
* [Software Structure](#software-structure)
* [Pipeline Structure](#pipeline-structure)
* [Installation and Setup](#installation-and-setup)
    * [Master](#master)
        * [Installation of software on Master node](#installation-of-software-on-master-node)
        * [Configure Jenkins](#configure-jenkins)
        * [Set up Cob-Pipeline specific configurations](#set-up-cob-pipeline-specific-configurations)
    * [Tarball Server](#tarball-server)
    * [Slaves](#slaves)
        * [Configure the node](#configure-the-node)
        * [Create a new slave node in Jenkins](#create-a-new-slave-node-in-jenkins-slave-setup-on-master)
    * [Manual Pipeline Generation (deprecated)](#manual-pipeline-generation-deprecated)


##Software Structure

For the usage of the Cob-Pipeline three parts are necessary:
* [Cob-Pipeline-Plugin](https://github.com/fmw-jk/cob-pipeline-plugin) for Jenkins<br/>
    This plugin allows the user to configure its individual build/test
    pipeline via the Jenkins web interface. Afterwards the automatic generation
    of the pipeline can be triggered.
* [jenkins\_setup repository](https://github.com/ipa320/jenkins_setup)<br/>
    This repository has to be available on the Jenkins server. It
    includes the code for the pipeline generation.
* [jenkins\_config repository](https://github.com/ipa320/jenkins_config)<br>
    In this repository all the pipeline configurations are stored.


##Pipeline Structure

The pipeline is made of multiple, differing Jenkins jobs which monitor the source code, build and test it in various envirements.
An authorized Jenkins user can configure its individual pipeline in its Jenkins user configurations.
The made configurations have to pass a validation and afterwards the automatic generation of the pipeline can be started.

A fully configured pipeline has always the structure shown in the picture below.
![Build-Pipeline structure](pictures/build_pipeline_structure.png "Structure of a Cob Build-Pipeline")

All build and test processes take place in [`chroot`s](help.ubuntu.com/community/BasicChroot) to garanty a clean and controlled environment.

###Job Types
####Starter Jobs
* **Pipestarter Job**<br/>
    Every *Pipestarter Job* polls one GitHub repository for source code changes.
    When a change is detected the *Priority-Build Job* gets triggered for the corresponding repository.

####Build Jobs
* **Priority-Build Job**<br/>
    The *Priority-Build Job* is the first real job in every pipeline.
    First of all it gets the corresponding `chroot` tarball for the environment to test the repository for from the tarball server.
    After entering the `chroot` the actual build process starts.
    * Clone the lastest version of the repository
    * Calculate its dependencies and install them
    * `make` the  repository and its dependencies
    At the end the `chroot` gets closed, archived in the tarball and uploaded to the tarball server.

* **Regular-Build Job**<br/>
    This job does the same as the *Priority-Build Job* but for more environments.

* **Downstream-Build Job**<br/>
    In contrast to the two previous build jobs the *Downstream-Build Job* builds the all ROS-packages that depend directly on the configured repository.

####Test Jobs
The following jobs run the tests given in the repository.
First of all the `chroot` tarball, created by the before executed *Build Job*, is downloaded.
The tests are executed inside this chroot.
* **Non-Graphics-Test Job**<br/>
    This job does only support tests which require no graphics support.
* **Graphics-Test Job**<br/>
    If graphics are required for the tests this job is the right one.

####Hardware Jobs
* **Hardware-Build Job**<br/>
    This job builds the code again on the selected hardware/robot.
    The environment (Ubuntu version, system architecture) is given by the hardware.

* **Hardware-Test Job**<br/>
    After a successful build the repository will closingly be tested on the hardware.


## [Jenkins Installation](README.md#jenkins-installation)

A description what has to be installed is given in the [short Jenkins Guide](README.md#jenkins-installation).

### Jenkins CI
This guide and the Jenkins plugin are designed for Jenkins v1.514.
[Here](README.md#up-or-downgrade-jenkins-to-version-v1514) is explained how to up- or downgrade.
> *!!! In general: Be careful with updating your Jenkins server. If you
> do, check if everything still works properly, especially the plugins!!!*

###Install an **apt-cacher** (optional):
During the later build process a lot packages will be installed.
If the build jobs run frequently, the network traffic increases quite much.
To limit the amount of packages to be downloaded from the internet and speed up the installation process a apt-cacher is pretty useful.
You can for example use the [apt-cacher-ng](http://www.unix-ag.uni-kl.de/~bloch/acng/).
To use the apt-cacher during the build process set up an apt-cacher and edit the [install_basics.sh script](./scripts/install_basics.sh) as descripted [here](./README.md#adapt-apt-cacher-address).

You can also use the apt-cacher of pbuilder. Then you should **NOT** do
[this](README.md#dont-use-pbuilders-aptcache).


## Jenkins configuration
To manage your Jenkins server, go to [http://localhost:8080/manage](http://localhost:8080/manage) or follow "Manage Jenkins" in the sidebar.
There you can configure everything.

###Configure Security
There are multiple ways to configure the global security of your Jenkins server.
First of all go to [**Configure Global Security**](http://localhost:8080/configureSecurity) and check *Enable Security*.

####Security Realm
The **Access Control** section gives the opportunity to select the **Security Realm** which defines how the users can login.

* **Jenkins's own user database**<br/>
    The easiest way is to use *Jenkins's own user database*.
    This option should always be available and possible.
    Now you can decide if every user can sign up (*Allow users to sign up*) or if the admin has to do this.

    If you use this, you have to create an user before you go on.
    This user will later on act as the admin user.
    Therefore save the configurations and **sign up** (upper right corner).
    Came back afterwards.

> * **Github Authentication Plugin**<br/>
>   Another way is to use the GitHub user database for user identification.
>   The [Github OAuth Plugin](#install-required-jenkins-plugins) has to be installed.
>   Configure the plugin as described
>   [here](https://wiki.jenkins-ci.org/display/JENKINS/Github+OAuth+Plugin) for an 'omnipotent' GitHub user.

> * **LDAP**<br/>
>   If a LDAP server is available, you can use it as the user database.
>   Therefore the [LDAP Plugin](#install-required-jenkins-plugins) is required.
>   How to configure the LDAP access can be found on the [plugin's website](https://wiki.jenkins-ci.org/display/JENKINS/LDAP+Plugin).
>   An example is given [here](pictures/LDAP_config.png).


####Authorization
In the **Authorization** subsection you can define the permission a specific user or a user group gets granted.
Therefore choose the 'Project-based Matrix Authorization Strategy'.

You have to give permissions to at least the *Anonymous* and the *authenticated* user group and an *admin* user.
The latter two have to be added to the matrix.

> **If you use [Jenkins's own user database](#jenkin's-own-user-database) the admin user you just created can be used.
> If one of the other [Security Realms](#security-realm) is used, take an existing user as admin.**

**The *admin* should have all rights.**
Otherwise you will [lock out yourself](https://wiki.jenkins-ci.org/display/JENKINS/Disable+security).
This account will also be used to create the pipeline jobs automatically.
The users and groups could get the permissions as shown below.
![Project-based Matrix Authorization Strategy](pictures/authentication.png "Example for Project-based Matrix Authorization Strategy")

Every user will automatically get the permission to see the workspace of all its own jobs.
For the 'Pipestarter' and 'Trigger' job it will also has 'Build'-permission.
> If you want to grant further permissions or give special permissions to individual users or user groups you can do it here.


### Basic configuration
The basic configurations of your Jenkins server are described in the [short Jenkins Guide](README.md#basic-configuration)


### Master node configuration
TODO


### Install the *cob-pipeline* plugin
Download the plugin (\*.hpi file) from [https://github.com/fmw-jk/cob-pipeline-plugin/releases](https://github.com/fmw-jk/cob-pipeline-plugin/releases), place it in `/var/lib/jenkins/plugins` and restart Jenkins.

    cd /var/lib/jenkins/plugins
    sudo wget https://github.com/fmw-jk/cob-pipeline-plugin/releases/download/v0.9.6/cob-pipeline.hpi
    sudo /etc/init.d/jenkins restart

Afterwards the plugin should be available and the **Pipeline Configuration** link should be present in the sidebar (see picture).

![sidebar](pictures/sidebar.png "sidebar with cob-pipeline-plugin")

Configure Jenkins as described below before you use the plugin.

### Install `jenkins_setup` & `jenkins_config`

> We assume that you work with an admin user account named 'jenkins'

All scripts and configurations will be stored in `/home/jenkins/jenkins-config`.

```bash
mkdir ~/jenkins-config
```

Setup ssh configuration (create ssh-key if it doesn't exist already and add github.com and localhost to known hosts).

```bash
ssh-keygen
touch ~/.ssh/known_hosts
ssh-keyscan -H github.com >> ~/.ssh/known_hosts
ssh-keyscan -H localhost >> ~/.ssh/known_hosts
```

You have to add this key to your GitHub 'omnipotent' user [http://github.com/settings/ssh](http://github.com/settings/ssh).

```bash
cat ~/.ssh/id_rsa.pub
```

Setup git configuration on master.

```bash
git config --global user.name "<USER_NAME>"
git config --global user.email "<EMAIL>"
```

Clone the `jenkins_setup` and `jenkins_config` repositories.
*You have to create a repository named 'jenkins_config'.*
> It is recommended to clone the
> [jenkins_setup](https://github.com/ipa320/jenkins_setup) repository to
> keep track of changes and updates.

```bash
git clone git@github.com:<GITHUB_USER>/jenkins_config.git ~/jenkins-config/jenkins_config
git clone git@github.com:<GITHUB_USER>/jenkins_setup.git ~/jenkins-config/jenkins_setup
```

Add the `jenkins_setup` module to the `$PYTHONPATH` (adapt the *ROS_RELEASE*).

```bash
sudo su -c 'echo "export PYTHONPATH=~/jenkins-config/jenkins_setup/src" > /etc/profile.d/python_path.sh'
sudo su -c 'echo "source /opt/ros/groovy/setup.sh" >> /etc/profile.d/python_path.sh'
```

Enable passwordless sudo rights for the jenkins user by adding the following line at the end of `/etc/sudoers` (open with `sudo visudo -f /etc/sudoers`).

```bash
jenkins    ALL=(ALL) NOPASSWD: ALL
```

Afterwards reboot the Jenkins-Server

```bash
sudo reboot now
```



### Jenkins plugin installation
####Install required Jenkins plugins
Go to [Jenkins plugin manager](http://localhost:8080/pluginManager/available) and install the following plugins:
* **Git Plugin** ([website](https://wiki.jenkins-ci.org/display/JENKINS/Git+Plugin))<br/>
    Monitors the repositories to build and triggers pipeline when a change is detected.
* **Parameterized Trigger Plugin** ([website](http://wiki.jenkins-ci.org/display/JENKINS/Parameterized+Trigger+Plugin))<br/>
    Is used to transfer build parameters from one job to the next.
    Here it is used to commit the repository to build or test.
* **Build Pipeline Plugin** ([website](http://code.google.com/p/build-pipeline-plugin))<br/>
    Provides a view where all pipeline jobs and their dependencies are shown.
    It also gives the opportunity to trigger the hardware test jobs manually.
* **Mailer** ([website](http://wiki.jenkins-ci.org/display/JENKINS/Mailer))<br/>
    Generates the email content depending on the build/test results and sends the emails.
* **View Job Filters**
  ([website](http://wiki.jenkins-ci.org/display/JENKINS/View+Job+Filters))<br/>
    Provides comprehensive possibilities to filter the jobs that can
    be seen by the specific user.

####Install supplementary Jenkins plugins
* *Matrix Reloaded Plugin*
  ([website](http://wiki.jenkins-ci.org/display/JENKINS/Matrix+Reloaded+Plugin))<br/>
    To start one or more entries of a matrix job.
* *LDAP Plugin* (not required but maybe useful)
  ([website](https://wiki.jenkins-ci.org/display/JENKINS/LDAP+Plugin))<br/>
    Authentication of users is delegated to a LDAP server.
* *Github OAuth Plugin* (not required but maybe useful)
  ([website](http://wiki.jenkins-ci.org/display/JENKINS/Github+OAuth+Plugin))<br/>
    Authentication of users is delegated to Github using the OAuth
    protocol.

___


####[Configure the default view](README.md#configure-default-view)

####[Configure the cob-pipeline plugin](README.md#configure-the-cob-pipeline-plugin)
How to configure the cob-pipeline plugin is described [here](README.md#configure-the-cob-pipeline-plugin).
You can follow this example.
Only **if you use the [Github Authentication Plugin](#security-realm) for authentication, enter the Jenkins Admin API token instead of its password.**
To get the API token go to the [admins user configuration](http://localhost:8080/me/configure).
It can be found in the section **API Token**. Press *Show API Token...*.


#####Mailer
######Default Subject
For example: ```$BUILD_STATUS: $PROJECT_NAME - Build # $BUILD_NUMBER!```
A complete list of tokens can be found at the help of the last entry
(Content Token Reference).
Do also [move the mailer template](#mailer-template) as described.

___


## Tarball Server:

The tarball server stores all the chroot tarball which will be used during the build process.
It can be the Jenkins master or another server.
In both cases you have to create a ```chroot_tarballs```-folder in `$HOME` which contains another folder where the used chroot tarballs will be stored:
```bash
mkdir -p ~/chroot_tarballs/in_use_on__<JENKINS_MASTER_NAME>
```

If you store the tarball on another server than your Jenkins master you have to enable a passwordless SSH connection between them both:
```bash
ssh-copy-id <master>            # _on tarball server_
ssh <master>                    # _on tarball server_
ssh-copy-id <tarball_server>    # _on master_
```

___

## Slaves:

Slaves are useful to distribute the load if many jobs get triggered and the run specific jobs on exclusive computers.

###Configure a build slave/node
To use a computer as Jenkins slave-node same preparations have to be done.
All configurations will be made for an admin user called 'jenkins'.

####Sudo commands without password on slave
To be able to run sudo commands without the need to enter the password each time, enter ```sudo visudo -f /etc/sudoers``` and add the following at the end of the `/etc/sudoers`-file:

    jenkins    ALL=(ALL) NOPASSWD: ALL

Exit with `CTRL-X`. After re-login you won't need a password anymore.

####Enable password-less ssh login from master to slave and slave to master.
The slave has to be able the access the master via SSH without a password (and the otherway around).
Enter the following command on each slave, login to the master and run the command again.

```bash
ssh-copy-id <master>    # _on slave_
ssh <master>            # _on slave_
ssh-copy-id <slave>     # _on master_
```

Go back with twice `CTRL-D`.

> If your tarball server differs from your Jenkins master, do the same for the tarball server.

####Pbuilder
Pbuilder is required! If not present, install it:
```bash
apt-get install pbuilder devscripts
```

#####Performance improvement
For the configurations a file called `~/.pbuilderrc` in the slaves `$HOME`-folder is
needed (`/etc/pbuilder/pbuilderrc` is an alternative).

######Don't use pbuilders aptcache
The aptcach of pbuilder is very useful but when the cache is getting
bigger gradually it takes quite a while to open a chroot from the
tarball. If you don't want to use it (for instance if you use an
external apt-cacher), add the following to
`~/.pbuilderrc`:
```conf
# don't use aptcache
APTCACHE=""
```

######Use ccache for build
To use ccache inside the pbuilder add the following to `~/.pbuilderrc`:
```conf
# ccache
sudo mkdir -p /var/cache/pbuilder/ccache
sudo chmod a+w /var/cache/pbuilder/ccache
export CCACHE_DIR="/var/cache/pbuilder/ccache"
export PATH="/usr/lib/ccache:${PATH}"
EXTRAPACKAGES=ccache
BINDMOUNTS="${CCACHE_DIR}"
```

######Use multi-core zipping
To speedup the zipping and unzipping of the chroot tarballs, install `pigz`:
```bash
apt-get install pigz
```

And add the following to .pbuilderrc:
```conf
# pigz; multicore zipping
COMPRESSPROG=pigz
```

######Mount memory to run the pbuilder chroots in it
Installations and builds inside the chroot need quite a lot write accesses. If you don't have a SSD installed, you can use the memory for this. Therefore you have to create a filesystem in your RAM, using `tmpfs` by adding the following to the slaves `/etc/fstab`:
```conf
# pbuilder
tmpfs   /var/cache/pbuilder/build   tmpfs   defaults,size=32000M    0   0
```

*The size depends on the size of the chroot you will work with (at least 3G, more is better). It can be larger then the RAM size. If the chroot size exceeds the RAM size it will use the SWAP as well.*

Additionally you have to add the following to `~/pbuilderrc`:
```conf
# tmpfs
APTCACHEHARDLINK=no
```

Finally mount `tmpfs` by entering **(as root)**:
```bash
mount -a
```

### Configure a hardware slave/node
A hardware slave is a computer where the hardware configuration/environment plays an important role, like a robot.
On such nodes run only [Hardware builds and test](#hardware-jobs) which use no `chroot` environment.
To configure such a node you have only to enable password-less [sudo commands](#enable-password-less-sudo-commands) and [SSH login](#enable-password-less-ssh-login-from-master-to-slave-and-slave-to-master) to the master (not the tarball server).


### Create a new slave node in Jenkins (Slave setup on master)

Go to [http://localhost:8080/computer](http://localhost:8080/computer) and add a *New Node*.
Name it and select *Dumb Slave*. *OK*.

- Set *# of executors* to `1`
- Set *Remote FS root* to the `$HOME`-Folder of the slave, e.g. `/home/jenkins`
- Set *Labels* to a combination of the following job names:

    ```
    prio_build regular_build update_tarballs
    prio_nongraphics_test regular_nongraphics_test
    prio_graphics_test regular_graphics_test
    downstream_build downstream_test
    ```

    Or if it's a hardware slave:

    ```
    hardware_build hardware_test
    ```

- Set *Host* to the slaves name.

*Save* and *Launch slave agent*.

___


> # Manual Pipeline Generation (deprecated):
>
> ## 1. Checkout this repository:
>
> Clone this repository to your desired location
> ```bash
> git clone git://github.com/ipa320/jenkins_setup.git <path to clone in>
> ```
>
>
> ## 2. Set up slave config file:
>
> Create a folder in your HOME-folder called: jenkins-config
> ```bash
> mkdir ~/jenkins-config
> ```
>
> Create a so called slave_config.yaml file with the following entries:
> ```yaml
> master: name_of_jenkins_master
> master_uri: "http://url_of_jenkins_master:8080"
> tarball_host: name_of_server_storing_the_chroot_tarballs
> tarball_folderpath: folder_the_tarballs_are_stored
> jenkins_login: user_name_with_right_to_create_jobs
> jenkins_pw: user_password
> ```
>
> ## 3. Add repository to PYTHONPATH:
>
> ```bash
> export PYTHONPATH=$PYTHONPATH:<repository_path>/src
> ```
>
> ## 4. Set up pipeline configuration:
>
> Checkout the repository [jenkins_config](https://github.com/ipa320/jenkins_config "ipa320/jenkins_config")
> ```bash
> git clone git@github.com:config/jenkins_config.git
> ```
>
> Repository structure:
> ```bash
> jenkins_config
> |-jenkins_master_name1
> | |- user_name1
> | |  |-pipeline_config.yaml
> | |- user_name2
> | |  |-pipeline_config.yaml
> |-jenkins_master_name2
> | |- user_name3
> | |  |-pipeline_config.yaml
> ```
>
> You have to create a folder according to your Jenkins masters name (if
> not existent yet). Inside create a folder with your user name. Within this
> folder set up a pipeline_config.yaml file with your configurations. You
> can use the \<jenkins_config_repository_location\>/jenkins-test-server/test-user
> as an example.
>
> When your done push it to GitHub.
>
> ## 5. Create pipeline:
>
> Execute the
> \<jenkins_setup_repository_location\>/scripts/generate_buildpipeline.py
> script to create all your pipeline jobs on the Jenkins CI Server.
>
> ```bash
> ./generate_buildpipeline.py <user_name>
> ```
