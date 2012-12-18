#!/usr/bin/env python

import optparse
from common import *


def main():
    parser = optparse.OptionParser()
    (options, args) = parser.parse_args()

    if len(args) <= 2:
        print "Usage: %s ros_distro repo1 repo2 ..." % sys.argv[0]
        raise BuildException("Wrong arbuments for build script")

    # get arguments
    ros_distro = args[0]
    repo_list = [args[i] for i in range(1, len(args))]
    workspace = os.environ['WORKSAPCE']

    print "Testing on ros distro %s" % ros_distro
    print "Testing the following repositories:"
    for repo in repo_list:
        print " - ", repo

    # update and upgrade
    print "\nUpdating chroot enviroment installed packages"
    call("apt-get update")
    call("apt-get dist-upgrade -y")

    # build depending on ros distro
    if ros_distro == "electric":
        build_electric(ros_distro, repo_list, workspace)
    else:
        build_post_electric(ros_distro, repo_list, workspace)


def build_electric(ros_distro, repo_list, workspace):
    pass


def build_post_electric(ros_distro, repo_list, workspace):
    # set up directories
    tmpdir = os.path.join('/tmp', 'test_repositories')
    repo_sourcespace = os.path.join(tmpdir, 'src_repository')
    dependson_sourcespace = os.path.join(tmpdir, 'src_depends_on')
    repo_buildspace = os.path.join(tmpdir, 'build_repository')
    dependson_buildspace = os.path.join(tmpdir, 'build_depend_on')

    # install Debian packages needed for script
    print "Installing Debian packages we need for running this script"
    call("apt-get install python-catkin-pkg python-rosinstall python-rosdistro --yes")
    import rosdistro


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
