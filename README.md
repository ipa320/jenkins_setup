# to be inserted at correct place
sudo apt-get install git-core





# Jenkins Guide

This repository contains the code (config, src and script files) to set up and run a Cob-Jenkins CI Server using the Cob-Pipeline-PlugIn.

## Prerequisites and assumptions
Before starting with this guide, please setup one machine with the following properties:
- Operation system: Ubuntu 12.04
- user: jenkins

assumptions:
- we're only using one machine which is master and slave at the same time
- apt-cacher is running on master




## Jenkins installation

### Debian packages "Debian/Ubuntu"
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

### Global Security
First off all go to [http://localhost:8080/configureSecurity](http://localhost:8080/configureSecurity)

The *Access Control* section gives the opportunity to select the *Security Realm* which defines how the users can login.

Check *Jenkins's own user database*. And check *Allow users to sign up*.

In the *Authorization* subsection you can define the permission a specific user or a user group gets granted.
Therefore choose the *Project-based Matrix Authorization Strategy*.

Now add an `admin`-user and give him all rights!

After click save the Server will throw you to a Login screen. Just register with the username of the admin you insert in
the table.

You have to give permissions to at least the anonymous and the authenticated user group and an admin user.
The latter two have to be added to the matrix.

Every user will automatically get permission to see the workspace of all its own jobs.
For the 'Pipestarter' and 'Trigger' job it will also get 'Build'-permission.

==== 3.2 Manage Jenkins

Under +http://localhost:8080/configure+ you can configure your Jenkins-Server.

- *# of executors* gives the number of simultaneously build job can start. *Set to 1*.
- *Jenkins URL*. Set to your servers name.

You can keep the default values for all other entries.

''''
=== Jenkins-PlugIns installation

Go to +http://localhost:8080/pluginManager/available+ and install the following plugins:

- https://wiki.jenkins-ci.org/display/JENKINS/Parameterized+Trigger+Plugin[*Parameterized Trigger PlugIn*]
- http://code.google.com/p/build-pipeline-plugin/[*Build Pipeline PlugIn*]
- https://wiki.jenkins-ci.org/display/JENKINS/Mailer[*Mailer*]
- https://wiki.jenkins-ci.org/display/JENKINS/View+Job+Filters[*View Job Filters*]
- https://wiki.jenkins-ci.org/display/JENKINS/Matrix+Reloaded+Plugin[*Matrix Reloaded PlugIn*]
- https://wiki.jenkins-ci.org/display/JENKINS/LDAP+Plugin[*LDAP PlugIn*]
- https://wiki.jenkins-ci.org/display/JENKINS/Github+OAuth+Plugin[*Github OAuth PlugIn*]

''''
=== IPA Configuration

==== 5.1 Robotic Operating System

Install ROS following +https://github.com/ipa-fmw-ak/Administration/wiki/05.-ROS-Guide+

==== 5.2 Master

===== 5.2.1 Create Cob-Pipeline Configuration Folder

All configurations should be stored in a common folder in the ^$HOME^ folder called +jenkins-config+.
----
mkdir ~/jenkins-config
----

===== 5.2.2 Git Configuration

Set up the GitHub user. This user has to have read-access to all repositories to build and write access to your ^jenkins_config^ repository.
----
git config --global user.name "<USER_NAME>"
git config --global user.email "<EMAIL>"
----

===== 5.2.3 SSH Configuration

A +.ssh+ folder is needed inside the +jenkins-config+ which contains a ssh-key to access the GitHub-repositories. Either you generate a new key with +ssh-keygen+ or you just copy the +~/.ssh+ of the master. You have to add this key to your GitHub user http://github.com/settings/ssh[SSH Settings]. This user should have read-access to all repositories you want to build. It is very important that 'github.com' belongs to the known hosts. Therefore the +.ssh+ folder should contain a ^known_hosts^ file. Whether 'github.com' is already known can be checked by entering:
----
ssh-keygen -H -f <known_hosts_PATH> -F github.com
----
If it is not known, you can add 'github.com' to the ^known_hosts^ by entering:
----
ssh-keyscan -H github.com > <known_hosts_PATH>
----
Furthermore the Jenkins masters SSH key itself has to be an authorized one.

===== 5.2.4 jenkins_config Repository

Clone the ^jenkins_config^ repository into the +jenkins-config+ folder.
----
git clone git@github.com:ipa320/jenkins_config.git ~/jenkins-config/jenkins_config
----

===== 5.2.5 jenkins_setup Repository

Clone the ^jenkins_setup^ repository into the +jenkins-config+ folder.
----
git clone git@github.com:ipa320/jenkins_setup.git ~/jenkins-config/jenkins_setup
----
_Adapt the GitHub user if you forked the repository!!!_

===== 5.2.6 PYTHONPATH

Add the ^jenkins_setup^ module to the +$PYTHONPATH+ (adapt the ROS_RELEASE).
----
echo "export PYTHONPATH=~/jenkins-config/jenkins_setup/src" > /etc/profile.d/python_path.sh
echo "source /opt/ros/<ROS_RELEASE>/setup.sh" >> /etc/profile.d/python_path.sh
----

Afterwards reboot the Jenkins-Server!

===== 5.2.7 Tarball Server

The tarball server stores all the chroot tarball which will be used during the build process. It can be the Jenkins master or another server. In both cases you have to create a ^chroot_tarballs^ folder in *$HOME* which contains another folder where the used chroot tarballs will be stored.
----
mkdir -p ~/chroot_tarballs/in_use_on__<JENKINS_MASTER_NAME>
----

==== 5.3 Slave

===== 5.3.1 Sudo Rights
To be able to run sudo commands without the need to enter the password each time:
----
sudo visudo -f /etc/sudoers
----
----
jenkins    ALL=(ALL) NOPASSWD: ALL
----

===== 5.3.2 SSH Access
The slave has to be able the access the master via SSH without a password (and the otherway around). Enter the following command on each slave, login to the master and run the command again.
----
ssh-copy-id <master>    # _on slave_
ssh <master>            # _on slave_
ssh-copy-id <slave>     # _on master_
----

===== 5.3.3 Pbuilder

----
sudo apt-get install pbuilder devscripts
----

*Performance Improvement*

For configurations a file called ^~/.pbuilderrc^ in the slaves +$HOME+ folder is needed (+/etc/pbuilder/pbuilderrc+ is an alternative).

*Pbuilders aptcache*

The aptcach of pbuilder is very useful but when the cache is getting bigger gradually it takes quite a while to open a chroot from the tarball. If you don't want to use it (for instance if you use an external apt-cacher), add the following to ^~/.pbuilderrc^:

----
# don't use aptcache
APTCACHE=""
----

*Use ccache for build*

To use ccache inside the pbuilder add the following to ^~/.pbuilderrc^:

----
# ccache
sudo mkdir -p /var/cache/pbuilder/ccache
sudo chmod a+w /var/cache/pbuilder/ccache
export CCACHE_DIR="/var/cache/pbuilder/ccache"
export PATH="/usr/lib/ccache:${PATH}"
EXTRAPACKAGES=ccache
BINDMOUNTS="${CCACHE_DIR}"
----

*Use multi-core zipping*

To speedup the zipping and unzipping of the chroot tarballs, install *pigz*.
----
sudo apt-get install pigz
----

And add the following to ^~/.pbuilderrc^:

----
# pigz; multicore zipping
COMPRESSPROG=pigz
----

*Mount memory to run the pbuilder chroots in it*

Installations and builds inside the chroot need quite a lot write accesses. If you don't have a SSD installed, you can use the memory for this. Therefore you have to create a filesystem in your RAM, using +tmpfs+ by adding the following to the slaves +/etc/fstab+:

----
# pbuilder
tmpfs   /var/cache/pbuilder/build   tmpfs   defaults,size=32000M    0   0
----

The size depends on the size of the chroot you will work with (at least 3G, more is better). It can be larger then the RAM size. If the chroot size exceeds the RAM size it will use the SWAP as well.

Additionally you have to add the following to ^~/pbuilderrc^:

----
# tmpfs
APTCACHEHARDLINK=no
----

Finally mount +tmpfs+ by entering (as root):

----
sudo mount -a
----

===== 5.3.4 Slave setup on Master
*TODO!!!*

==== 5.4 The Cob-Pipeline

For the usage of the Cob-Pipeline three parts are necessary:

- https://github.com/fmw-jk/cob-pipeline-plugin[Cob-Pipeline-Plugin]

This plugin allows the user to configure its individual build/test pipeline via the Jenkins web interface. Afterwards the automatic generation of the pipeline can be triggered.

- https://github.com/ipa320/jenkins_setup[jenkins_setup repository]

This repository has to be available on the Jenkins server. It includes the code for the pipeline generation.

- https://github.com/ipa320/jenkins_config[jenkins_config repository]

In this repository all the pipeline configurations are stored.

===== 5.4.1 Install the Cob-Pipeline

Download the *.hpi* file from +https://github.com/fmw-jk/cob-pipeline-plugin/releases+
and place it in +<JENKINS_HOME>/plugins+.
----
cd /var/lib/jenkins/plugins
sudo wget https://github.com/fmw-jk/cob-pipeline-plugin/releases/download/v0.9.5-alpha/cob-pipeline.hpi
----

Restart your Jenkins-Server
----
sudo /etc/init.d/jenkins restart
----

===== 5.4.2 Configure the Cob-Pipeline

Go to the Cob Pipeline Configuration section. The following fields are all required for the use.

- *Jenkins Admin Login/Password*

This is the user you configured before in the Configure Security part with all the permissions. Enter its login name and password.
    
- *Configuration Folder*
    
Enter the path of the Cob-Pipeline configuration folder.
    
- *Tarball Location*
    
Enter the location where the tarballs are stored.
    
- *GitHub User Login/Password*
    
This is the user that has read-permission to all the repositories you want to be tested. It has also write-permission to your jenkins-config repository.
    
- *Pipeline Repositories Owner/Fork*
    
GitHub user that ownes the ^jenkins_setup^ and the ^jenkins_config^ repository.
    
- *ROS Releases*
    
ROS versions that should be supported by your build/test pipeline.
    
- *Robots*
    
Nodes which can be chosen for Hardware Build/Test jobs.
    
- *Target Platform Url*
    
URL where the ROS ^targets.yaml^ is stored, defining the Ubuntu target platforms for each ROS Version, e.g..

_When you fill out the fields, the values will be validated in the background._

===== 5.4.3 Pipeline Structure

The pipeline is made of multiple, differing Jenkins jobs which monitor the source code or build and test it in various envirements. An authorized Jenkins user can configure its individual pipeline in its Jenkins user configurations. The made configurations have to pass a validation and afterwards the automatic generation of the pipeline can be started.

A fully configured pipeline has always this structure:

* Starter Jobs

.. Pipestarter Job

* Build Jobs

.. Priority-Build Job

.. Regular-Build Job

.. Downstream-Build Job

* Test Jobs

.. Non-Graphics-Test Job

.. Graphics-Test Job

* Hardware Jobs

.. Hardware-Build Job

.. Hardware-Test Job

===== 5.4.4 Manual Pipeline Generation

*Checkout this Repository*

Clone this repository to your desired location.
----
$ git clone git://github.com/ipa320/jenkins_setup.git <path to clone in>
----

*Setup Slave Config-File*

Create a folder in your +$HOME+ folder called +jenkins-config+
----
$ mkdir ~/jenkins-config
----

Create a so called ^slave_config.yaml^ file with the following entries:
----
master: name_of_jenkins_master
master_uri: "http://url_of_jenkins_master:8080"
tarball_host: name_of_server_storing_the_chroot_tarballs
tarball_folderpath: folder_the_tarballs_are_stored
jenkins_login: user_name_with_right_to_create_jobs
jenkins_pw: user_password
----

*Add Repository to PYTHONPATH*

----
$ export PYTHONPATH=$PYTHONPATH:<repository_path>/src
----

*Setup Pipeline Configuration

Checkout the repository ^jenkins_config^
----
$ git clone git@github.com:config/jenkins_config.git
----

Repository structure:
----
jenkins_config
|-jenkins_master_name1
| |-user_name1
| | |-pipeline_config.yaml
| |-user_name2
| | |-pipeline_config.yaml
|-jenkins_master_name2
| |-user_name3
| | |-pipeline_config.yaml
----

You have to create a folder according to your Jenkins masters name (if not existent yet). Inside create a folder with your user name. Within this folder set up a ^pipeline_config.yaml^ file with your configurations. You can use the <jenkins_config_repository_location>/jenkins-test-server/test-user as an example.

_When your done push it to GitHub!_

*Create Pipeline*

Execute the ^<jenkins_setup_repository_location>/scripts/generate_buildpipeline.py^ script to create all your pipeline jobs on the Jenkins CI Server.
----
$ ./generate_buildpipeline.py <user_name>
----

==== 5.5 PlugIns

Went to +http://localhost:8080/pluginManager/available+

- https://wiki.jenkins-ci.org/display/JENKINS/Parameterized+Trigger+Plugin[*Parameterized Trigger PlugIn*]

Is used to transfer build parameters from one job to the next. Here it is used to commit the repository to build or test.

- http://code.google.com/p/build-pipeline-plugin/[*Build Pipeline PlugIn*]

Provides a view where all pipeline jobs and their dependencies are shown. It also gives the opportunity to trigger the hardware test jobs manually.

- https://wiki.jenkins-ci.org/display/JENKINS/Mailer[*Mailer*]

Generates the email content depending on the build/test results and sends the emails.

_CONFIGURATION_:

A template for the Mailer plugin is provided in https://github.com/ipa320/jenkins_setup/blob/master/templates/email-templates/html-with-health-builds-tests.jelly[this] repository. Copy it into +$JENKINS_HOME/email-templates+ (+$JENKINS_HOME+ is usually +/var/lib/jenkins+). You can adapt the template to your requirements.

- https://wiki.jenkins-ci.org/display/JENKINS/View+Job+Filters[*View Job Filters*]

Provides comprehensive possibilities to filter the jobs that can be seen by the specific user.

- https://wiki.jenkins-ci.org/display/JENKINS/Matrix+Reloaded+Plugin[*Matrix Reloaded PlugIn*]

To start one or more entries of a matrix job.

- https://wiki.jenkins-ci.org/display/JENKINS/LDAP+Plugin[*LDAP PlugIn*]

Authentication of users is delegated to a LDAP server.

- https://wiki.jenkins-ci.org/display/JENKINS/Github+OAuth+Plugin[*Github OAuth PlugIn*]

Authentication of users is delegated to Github using the OAuth protocol.

_More PlugIns will follow..._

==== 5.6 Global and Individual Project List Views
*TODO!!!*

==== 5.7 Backup your Jenkins-Server
*TODO!!!*

