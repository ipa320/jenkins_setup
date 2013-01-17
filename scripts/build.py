#!/usr/bin/env python

import optparse
import sys
import os

from jenkins_setup import cob_common, cob_distro, rosdep


def main():
    # parse parameter values
    parser = optparse.OptionParser()
    (options, args) = parser.parse_args()

    if len(args) < 4:
        print "Usage: %s server_name user_name ros_distro build_repo" % sys.argv[0]
        raise cob_common.BuildException("Wrong arguments for build script")

    # get arguments
    server_name = args[0]
    user_name = args[1]
    ros_distro = args[2]
    build_repo = args[3]  # repository to build
    workspace = os.environ['WORKSPACE']

    # (debug) output
    print "\n", 50 * 'X'
    print "\nTesting on ros distro: %s" % ros_distro
    print "Testing the repository: %s" % build_repo
    print "\n", 50 * 'X'

    # update sourcelist and upgrade installed basic packages
    print "\nUpdating chroot enviroment installed packages"
    cob_common.call("apt-get update")
    cob_common.call("apt-get dist-upgrade -y")

    print "\nUpdating rosinstall"
    cob_common.call("pip install -U rosinstall")

    # clone jenkins_config repository
    #print "\nCloning jenkins_config repository"  # TODO necessary??
    #cob_common.call("git clone git://github.com/fmw-jk/jenkins_config.git %s/jenkins_config" % workspace)  # TODO change to ipa320
    #cob_common.call("cp -r %s/jenkins_config/%s/%s %s/pipeline_config_dir" % (workspace, server_name, user_name, workspace))

    # cob_distro_pipe object
    cdp_instance = cob_distro.Cob_Distro_Pipe()
    cdp_instance.load_from_url(server_name, user_name)
    buildpipe_repos = cdp_instance.repositories

    # build depending on ros_ distro
    if ros_distro == "electric":
        build_electric(ros_distro, build_repo, buildpipe_repos, workspace)
    else:
        build_post_electric(ros_distro, build_repo, buildpipe_repos, workspace)


def build_electric(ros_distro, build_repo, buildpipe_repos, workspace):
    # set up directories variables
    tmpdir = os.path.join('/tmp', 'test_repositories')
    repo_sourcespace = os.path.join(tmpdir, 'src_repository')

    # install Debian packages needed for script TODO ??
    #print "Installing Debian packages we need for running this script"
    #cob_common.call("apt-get install python-catkin-pkg python-rosdistro --yes")

    # download build_repo from source
    print "Creating rosinstall file for repo"
    rosinstall = ""
    if build_repo in buildpipe_repos:
        rosinstall += buildpipe_repos[build_repo].get_rosinstall()
    else:
        raise cob_common.BuildException("Pipeline was triggered by repo %s which is \
                                         not in pipeline config!" % build_repo)

    print "Rosinstall file for repository: \n %s" % rosinstall
    # write .rosinstall file
    with open(os.path.join(workspace, "repo.rosinstall"), 'w') as f:
        f.write(rosinstall)
    print "Install repository from source:"
    # create repo sourcespace directory 'src_repository'
    os.makedirs(repo_sourcespace)
    # rosinstall repos
    # TODO handle dry stacks
    cob_common.call("rosinstall %s %s/repo.rosinstall /opt/ros/%s"
                    % (repo_sourcespace, workspace, ros_distro))

    # TODO get deps, build and test
    # get the repositories build dependencies
    # TODO handle dry stacks
    print "Get build dependencies of repo"
    #repo_build_wet_dependencies = cob_common.get_dependencies(repo_sourcespace, build_depends=True, test_depends=False)
    #print "Found wet dependencies:\n%s" % '- ' + '\n- '.join(repo_build_dependencies)
    (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace)
    repo_build_wet_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {})
    print "Found wet dependencies:\n%s" % '- ' + '\n- '.join(repo_build_wet_dependencies)
    repo_build_dry_dependencies = cob_common.get_nonlocal_dependencies({}, stacks, manifest_packages)
    print "Found dry dependencies:\n%s" % '- ' + '\n- '.join(repo_build_dry_dependencies)
    repo_build_stack_dependencies = cob_common.get_nonlocal_dependencies({}, stacks, {})
    print "Found stack dependencies:\n%s" % '- ' + '\n- '.join(repo_build_stack_dependencies)
    repo_build_dependencies = repo_build_stack_dependencies
    # install user-defined/customized dependencies from source
    rosinstall = ''
    fulfilled_deps = []
    for dep in repo_build_dependencies:
        if dep in buildpipe_repos[build_repo].dependencies:
            if buildpipe_repos[build_repo].dependencies[dep].poll:
                print "Install user-defined build dependency %s from source" % dep
                rosinstall += buildpipe_repos[build_repo].dependencies[dep].get_rosinstall()
                repo_build_dependencies.remove(dep)
                fulfilled_deps.append(dep)

    if rosinstall != '':
        print "Rosinstall file for user-defined build dependencies: \n %s" % rosinstall
        # write .rosinstall file
        with open(os.path.join(workspace, "repo.rosinstall"), 'w') as f:
            f.write(rosinstall)
        print "Install user-defined build dependencies from source"
        # rosinstall depends
        # TODO handle dry stacks
        cob_common.call("rosinstall %s %s/repo.rosinstall --catkin"
                        % (repo_sourcespace, workspace))

        # get also deps of just installed user-defined/customized dependencies
        (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace)
        repo_build_stack_dependencies = cob_common.get_nonlocal_dependencies({}, stacks, {})
        repo_build_dependencies = [dep for dep in repo_build_stack_dependencies if dep not in fulfilled_deps]

    repo_build_dependencies_apt = ['ros-electric-' + dep.replace('_', '-') for dep in repo_build_dependencies]

    print "Install build dependencies of repo list: %s" % (', '.join(repo_build_dependencies))
    cob_common.apt_get_install(repo_build_dependencies_apt)

    print "Rosdep"
    cob_common.call("rosmake rosdep")
    for stack in stacks.keys():
        cob_common.call("rosdep install -y %s" % stack)


def build_post_electric(ros_distro, build_repo, buildpipe_repos, workspace):
    # set up directories variables
    tmpdir = os.path.join('/tmp', 'test_repositories')
    repo_sourcespace = os.path.join(tmpdir, 'src_repository')
    #dependson_sourcespace = os.path.join(tmpdir, 'src_depends_on')
    repo_buildspace = os.path.join(tmpdir, 'build_repository')
    #dependson_buildspace = os.path.join(tmpdir, 'build_depend_on')

    # install Debian packages needed for script
    print "Installing Debian packages we need for running this script"
    cob_common.call("apt-get install python-catkin-pkg python-rosdistro --yes")

    # download build_repo from source
    print "Creating rosinstall file for repo"
    rosinstall = ""
    # check if triggering repo is defined in buildpipe config
    # TODO
    if build_repo in buildpipe_repos:
        rosinstall += buildpipe_repos[build_repo].get_rosinstall()
    else:
        raise cob_common.BuildException("Pipeline was triggered by repo %s which is \
                                         not in pipeline config!" % build_repo)

    print "Rosinstall file for repository: \n %s" % rosinstall
    # write .rosinstall file
    with open(os.path.join(workspace, "repo.rosinstall"), 'w') as f:
        f.write(rosinstall)
    print "Installing repository from source:"
    # create repo sourcespace directory 'src_repository'
    os.makedirs(repo_sourcespace)
    # rosinstall repos
    # TODO handle dry stacks
    cob_common.call("rosinstall %s %s/repo.rosinstall --catkin"
                    % (repo_sourcespace, workspace))

    # get the repositories build dependencies
    # TODO handle dry stacks
    print "Get build dependencies of repo"
    #repo_build_dependencies = cob_common.get_dependencies(repo_sourcespace, build_depends=True, test_depends=False)
    #print "Found dependencies:\n%s" % '- ' + '\n- '.join(repo_build_dependencies)
    repo_build_wet_dependencies = cob_common.get_dependencies(repo_sourcespace, build_depends=True, test_depends=False)
    print "Found wet dependencies:\n%s" % '- ' + '\n- '.join(repo_build_wet_dependencies)
    repo_build_dependencies = repo_build_wet_dependencies
    (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace)
    repo_build_wet_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {})
    print "Found wet dependencies:\n%s" % '- ' + '\n- '.join(repo_build_wet_dependencies)
    repo_build_dry_dependencies = cob_common.get_nonlocal_dependencies({}, stacks, manifest_packages)
    print "Found dry dependencies:\n%s" % '- ' + '\n- '.join(repo_build_dry_dependencies)
    # install user-defined/customized dependencies from source
    rosinstall = ''
    for dep in repo_build_dependencies:
        if dep in buildpipe_repos[build_repo].dependencies:
            if buildpipe_repos[build_repo].dependencies[dep].poll:
                print "Install user-defined build dependency %s from source" % dep
                rosinstall += buildpipe_repos[build_repo].dependencies[dep].get_rosinstall()
                repo_build_dependencies.remove(dep)

    if rosinstall != '':
        print "Rosinstall file for user-defined build dependencies: \n %s" % rosinstall
        # write .rosinstall file
        with open(os.path.join(workspace, "repo.rosinstall"), 'w') as f:
            f.write(rosinstall)
        print "Install user-defined build dependencies from source"
        # rosinstall depends
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
    #ros_env_repo = cob_common.get_ros_env(os.path.join(repo_buildspace, 'devel/setup.bash'))

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
