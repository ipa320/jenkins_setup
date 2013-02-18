jenkins\_setup
=============

This repository contains the code (config, src and script files) to setup and run a
cob-Jenkins CI server.


Generate pipeline (workaround):
===============================

1. Checkout this repository:
----------------------------

Clone this repository to your desired location
```bash
git clone git://github.com/ipa320/jenkins_setup.git <path to clone in>
```


2. Set up slave config file:
----------------------------

Create a folder in your HOME-folder called: jenkins-config
```bash
mkdir ~/jenkins-config
```

Create the so called slave_config.yaml file with the following entries:
```yaml
master: name_of_jenkins_master
master_uri: "http://url_of_jenkins_master:8080"
tarball_host: name_of_server_storing_the_chroot_tarballs
tarball_folderpath: folder_the_tarballs_are_stored
jenkins_login: user_name_with_right_to_create_jobs
jenkins_pw: user_password
```

3. Add repository to PYTHONPATH:
--------------------------------

```bash
export PYTHONPATH=$PYTHONPATH:<repository_path>/src
```

4. Set up pipeline configuration:
---------------------------------

Checkout the repository ipa320/jenkins_config
```bash
git clone git@github.com:config/jenkins_config.git
```

Repository structure:
jenkins_config
|-jenkins_master_name1
| |- user_name1
| |  |-pipeline_config.yaml
| |- user_name2
| |  |-pipeline_config.yaml
|-jenkins_master_name2
| |- user_name3
| |  |-pipeline_config.yaml

You have to create a folder according to your Jenkins masters name (if
necessary). Inside create a folder with your user name. Within this
folder set up a pipeline_config.yaml file with your configurations. You
can use the <jenkins_config_repository_location>/jenkins-test-server/test-user
as an example.

When your done push it to GitHub.

5. Create pipeline:
-------------------

Execute the
<jenkins_setup_repository_location>/scripts/generate_buildpipeline.py
script to create all your pipeline jobs on the Jenkins CI Server.

```bash
./generate_buildpipeline.py <user_name>
```
