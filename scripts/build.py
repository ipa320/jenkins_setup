#!/usr/bin/env python

import optparse
import sys
import os

from jenkins_setup import cob_common, cob_develdistro, rosdep


def main():
    # parse parameter values
    parser = optparse.OptionParser()
    (options, args) = parser.parse_args()

    if len(args) < 3:
        print "Usage: %s pipeline_name ros_distro repo1 repo2 ..." % sys.argv[0]
        raise cob_common.BuildException("Wrong arguments for build script")

    # get arguments
    pipeline_name = args[0]
    ros_distro = args[1]
    repo_list = [args[i] for i in range(2, len(args))]  # repositories to build
    workspace = os.environ['WORKSPACE']

    # (debug) output
    print "\nTesting on ros distro %s" % ros_distro
    print "Testing the following repositories:"
    for repo in repo_list:
        print " - ", repo

    # update sourcelist and upgrade installed basic packages
    print "\nUpdating chroot enviroment installed packages"
    cob_common.call("apt-get update")
    cob_common.call("apt-get dist-upgrade -y")

    # clone jenkins_config repository
    print "\nCloning jenkins_config repository"
    cob_common.call("git clone git://github.com/fmw-jk/jenkins_config.git %s/jenkins_config" % workspace)  # TODO change to ipa320
    cob_common.call("cp -r %s/jenkins_config/%s %s/pipeline_config_dir" % (workspace, pipeline_name, workspace))

    # build depending on ros_ distro
    if ros_distro == "electric":
        build_electric(pipeline_name, ros_distro, repo_list, workspace)
    else:
        build_post_electric(pipeline_name, ros_distro, repo_list, workspace)


def build_electric(pipeline_name, ros_distro, repo_list, workspace):
    pass


def build_post_electric(pipeline_name, ros_distro, repo_list, workspace):
    # set up directories variables
    tmpdir = os.path.join('/tmp', 'test_repositories')
    repo_sourcespace = os.path.join(tmpdir, 'src_repository')
    #dependson_sourcespace = os.path.join(tmpdir, 'src_depends_on')
    repo_buildspace = os.path.join(tmpdir, 'build_repository')
    #dependson_buildspace = os.path.join(tmpdir, 'build_depend_on')

    # install Debian packages needed for script
    print "Installing Debian packages we need for running this script"
    cob_common.call("apt-get install python-catkin-pkg python-rosinstall python-rosdistro --yes")

    # get buildpipeline configurations from yaml file hosted on github repo
    # jenkins_config
    buildpipe_configs = cob_common.get_buildpipeline_configs(pipeline_name)  # TODO username and server needed ??

    # download repo_list from source
    print "Creating rosinstall file for repo list"
    buildpipe_repos = cob_develdistro.Cob_Distro(ros_distro, buildpipe_configs['repositories'])
    rosinstall = ""
    for repo in repo_list:
        if repo in buildpipe_configs['repositories']:
            rosinstall += buildpipe_repos.repositories[repo].get_rosinstall()  # TODO
        else:
            raise cob_common.BuildException("Pipeline was triggered by repo %s which is \
                                             not in pipeline config:\n%s"
                                            % (repo, buildpipe_configs['repositories']))

    print "Rosinstall file for all repositories: \n %s" % rosinstall
    # write .rosinstall file
    with open(os.path.join(workspace, "repo.rosinstall"), 'w') as f:
        f.write(rosinstall)
    print "Install repository list from source"
    # create repo sourcespace directory 'src_repository'
    os.makedirs(repo_sourcespace)
    # rosinstall repos
    # TODO handle private repos
    # TODO handle dry stacks
    cob_common.call("rosinstall %s %s/repo.rosinstall --catkin"
                    % (repo_sourcespace, workspace))

    # get the repositories build dependencies
    # TODO handle dry stacks
    print "Get build dependencies of repo list"
    repo_build_dependencies = cob_common.get_dependencies(repo_sourcespace, build_depends=True, test_depends=False)
    # install user-defined dependencies from source
    rosinstall = ''
    for name, data in buildpipe_repos.repositories.iteritems():
        if data.dep:
            if name in repo_build_dependencies:
                rosinstall += buildpipe_repos.repositories[name].get_rosinstall()
                repo_build_dependencies.remove(name)
    if rosinstall != '':
        print "Rosinstall file for user-defined build dependencies: \n %s" % rosinstall
        # write .rosinstall file
        with open(os.path.join(workspace, "repo.rosinstall"), 'w') as f:
            f.write(rosinstall)
        print "Install user-defined build dependencies from source"
        # rosinstall depends
        # TODO handle private repos
        # TODO handle dry stacks
        cob_common.call("rosinstall %s %s/repo.rosinstall --catkin"
                        % (repo_sourcespace, workspace))

    # Create rosdep object
    print "Create rosdep object"
    rosdep_resolver = rosdep.RosDepResolver(ros_distro)

    print "Install build dependencies of repo list: %s" % (', '.join(repo_build_dependencies))
    cob_common.apt_get_install(repo_build_dependencies, rosdep_resolver)

    # replace the CMakeLists.txt file for repositories that use catkin
    print "Removing the CMakeLists.txt file generated by rosinstall"
    os.remove(os.path.join(repo_sourcespace, 'CMakeLists.txt'))
    print "Create a new CMakeLists.txt file using catkin"
    ros_env = cob_common.get_ros_env('/opt/ros/%s/setup.bash' % ros_distro)
    cob_common.call("catkin_init_workspace %s" % repo_sourcespace, ros_env)
    os.makedirs(repo_buildspace)
    os.chdir(repo_buildspace)
    cob_common.call("cmake %s" % repo_sourcespace, ros_env)
    ros_env_repo = cob_common.get_ros_env(os.path.join(repo_buildspace, 'devel/setup.bash'))

    # build repositories and tests
    print "Build repo list"
    cob_common.call("make", ros_env)
    cob_common.call("make tests", ros_env)

    # get the repositories test and run dependencies
    print "Get test and run dependencies of repo list"
    repo_test_dependencies = cob_common.get_dependencies(repo_sourcespace, build_depends=False, test_depends=True)
    print "Install test and run dependencies of repo list: %s" % (', '.join(repo_test_dependencies))
    cob_common.apt_get_install(repo_test_dependencies, rosdep_resolver)

    # run tests
    print "Test repo list"
    cob_common.call("make run_tests", ros_env)

    # copy test results
    cob_common.copy_test_results(workspace, repo_buildspace)

#    #TODO used when get dependencies
#    import rosdistro
#    from jenkins_setup import cob_rosdistro
#
#    # parse the cobdistro file
#    print "Parsing cobdistro file for %s" % ros_distro
#    cob_distro = cob_rosdistro.CobRosDistro(ros_distro)
#
#    # parse the rosdistro file
#    print "Parsing rosdistro file for %s" % ros_distro
#    distro = rosdistro.RosDistro(ros_distro)
#    print "Parsing devel file for %s" % ros_distro
#    devel = rosdistro.DevelDistro(ros_distro)


if __name__ == "__main__":
    # global try
    try:
        main()
        print "Build script finished cleanly!"

    # global catch
    except cob_common.BuildException as ex:
        print ex.msg

    except Exception as ex:
        print "Build script failed! Check out the console output above for details."
        raise ex
