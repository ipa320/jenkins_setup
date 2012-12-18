#!/usr/bin/env python

import optparse
from jenkins_setup.cob_common import *


def main():
    parser = optparse.OptionParser()
    (options, args) = parser.parse_args()

    if len(args) <= 3:
        print "Usage: %s pipeline_name ros_distro repo1 repo2 ..." % sys.argv[0]
        raise BuildException("Wrong arbuments for build script")

    # get arguments
    pipeline_name = args[0]
    ros_distro = args[1]
    repo_list = [args[i] for i in range(2, len(args))]
    workspace = os.environ['WORKSPACE']

    print "\nTesting on ros distro %s" % ros_distro
    print "Testing the following repositories:"
    for repo in repo_list:
        print " - ", repo

    # update and upgrade
    print "\nUpdating chroot enviroment installed packages"
    call("apt-get update")
    call("apt-get dist-upgrade -y")

    # clone jenkins_config
    print "\nCloning jenkins_config repository"
    call("git clone git@github.com:fmw-jk/jenkins_config.git %s/jenkins_config" % workspace)  # TODO change to ipa320
    call("cp -r %s/jenkins_config/%s %s/pipeline_config_dir" % (workspace, pipeline_name, workspace))

    # build depending on ros distro
    if ros_distro == "electric":
        build_electric(pipeline_name, ros_distro, repo_list, workspace)
    else:
        build_post_electric(pipeline_name, ros_distro, repo_list, workspace)


def build_electric(pipeline_name, ros_distro, repo_list, workspace):
    pass


def build_post_electric(pipeline_name, ros_distro, repo_list, workspace):
    # set up directories
    tmpdir = os.path.join('/tmp', 'test_repositories')
    repo_sourcespace = os.path.join(tmpdir, 'src_repository')
    dependson_sourcespace = os.path.join(tmpdir, 'src_depends_on')
    repo_buildspace = os.path.join(tmpdir, 'build_repository')
    dependson_buildspace = os.path.join(tmpdir, 'build_depend_on')

    # install Debian packages needed for script
    print "Installing Debian packages we need for running this script"
    call("apt-get install python-catkin-pkg python-rosinstall python-rosdistro --yes")

    buildpipe_configs = get_buildpipeline_configs(pipeline_name)  # TODO username and server needed

    # download repo_list from source
    print "Creating rosinstall fiel for repo list"
    rosinstall = ""
    for repo in repo_list:
        if repo not in buildpipe_configs['repositories']:
            raise BuildException("Pipeline was triggered by repo %s which is \
                                 not in pipeline config:\n%s"
                                 % (repo, buildpipe_configs['repositories']))
        else:
            rosinstall += buildpipe_configs['repositories']  # TODO


    #TODO used when get dependencies
    import rosdistro
    from jenkins_setup import cob_rosdistro

    # parse the cobdistro file
    print "Parsing cobdistro file for %s" % ros_distro
    cob_distro = cob_rosdistro.CobRosDistro(ros_distro)

    # parse the rosdistro file
    print "Parsing rosdistro file for %s" % ros_distro
    distro = rosdistro.RosDistro(ros_distro)
    print "Parsing devel file for %s" % ros_distro
    devel = rosdistro.DevelDistro(ros_distro)


if __name__ == "__main__":
    # global try
    try:
        main()
        print "Build script finished cleanly!"

    # global catch
    except BuildException as ex:
        print ex.msg

    except Exception as ex:
        print "Build script failed! Check out the console output above for details."
        raise ex
