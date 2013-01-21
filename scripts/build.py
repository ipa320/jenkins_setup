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
    test_results_dir = os.path.join(repo_sourcespace, 'test_results')

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
    cob_common.call("rosinstall %s %s/repo.rosinstall /opt/ros/%s"
                    % (repo_sourcespace, workspace, ros_distro))

    # get the repositories build dependencies
    print "Get build dependencies of repo"
    #repo_build_wet_dependencies = cob_common.get_dependencies(repo_sourcespace, build_depends=True, test_depends=False)
    #print "Found wet dependencies:\n%s" % '- ' + '\n- '.join(repo_build_dependencies)
    (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace)
    repo_build_wet_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {})
    print "Found wet dependencies:\n%s" % '- ' + '\n- '.join(repo_build_wet_dependencies)
    repo_build_stack_dependencies = cob_common.get_nonlocal_dependencies({}, stacks, {})
    print "Found stack dependencies:\n%s" % '- ' + '\n- '.join(repo_build_stack_dependencies)
    repo_build_dependencies = repo_build_stack_dependencies
    # install user-defined/customized dependencies from source
    rosinstall = ''
    fulfilled_deps = []
    for dep in repo_build_dependencies:
        if dep in buildpipe_repos[build_repo].dependencies:
            print "Install user-defined build dependency %s from source" % dep
            rosinstall += buildpipe_repos[build_repo].dependencies[dep].get_rosinstall()
            fulfilled_deps.append(dep)

    # check if all user-defined/customized dependencies are satisfied
    if sorted(fulfilled_deps) != sorted(buildpipe_repos[build_repo].dependencies):
        print "Not all user-defined build dependencies are fulfilled"
        print "User-defined build dependencies:\n%s" % '- ' + '\n- '.join(buildpipe_repos[build_repo].dependencies)
        print "Fulfilled dependencies:\n%s" % '- ' + '\n- '.join(fulfilled_deps)
        raise cob_common.BuildException("Not all user-defined build dependencies are fulfilled")

    if rosinstall != '':
        # write .rosinstall file
        print "Rosinstall file for user-defined build dependencies: \n %s" % rosinstall
        with open(os.path.join(workspace, "repo.rosinstall"), 'w') as f:
            f.write(rosinstall)
        print "Install user-defined build dependencies from source"
        # rosinstall depends
        cob_common.call("rosinstall %s %s/repo.rosinstall /opt/ros/%s"
                        % (repo_sourcespace, workspace, ros_distro))

        # get also deps of just installed user-defined/customized dependencies
        (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace)
        repo_build_stack_dependencies = cob_common.get_nonlocal_dependencies({}, stacks, {})
        repo_build_dependencies = [dep for dep in repo_build_stack_dependencies if dep not in fulfilled_deps]

    repo_build_dependencies_apt = ['ros-electric-' + dep.replace('_', '-') for dep in repo_build_dependencies]

    print "Install build dependencies of repo list: %s" % (', '.join(repo_build_dependencies))
    cob_common.apt_get_install(repo_build_dependencies_apt)

    # TODO
    print "Env"
    ros_env = cob_common.get_ros_env('/opt/ros/%s/setup.bash' % ros_distro)
    cob_common.call("env", ros_env)
    ros_env_repo = cob_common.get_ros_env(os.path.join(repo_sourcespace, 'setup.bash'))
    cob_common.call("env", ros_env_repo)
    print "Rosdep"
    cob_common.call("rosmake rosdep", ros_env)
    for stack in stacks.keys():
        cob_common.call("rosdep install -y %s" % stack, ros_env_repo)

    # TODO build (like in hudson_helper)
    # build repositories and tests
    print "Build repo"
    cob_common.call("rosmake --pjobs=8 --output=%s %s" % (test_results_dir, build_repo), ros_env)
    os.makedirs(test_results_dir)
    cob_common.call("rosmake --pjobs=8 --test-only --output=%s %s" % (test_results_dir, build_repo), ros_env)
    # TODO output dir ??
    # copy test results
    cob_common.call("rosrun rosunit clean_junit_xml.py", ros_env)
    cob_common.copy_test_results(workspace, repo_sourcespace)


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

    ### DEBUG ###
    # get deps directly for catkin like willow
    try:
        repo_build_dependencies = cob_common.get_dependencies(repo_sourcespace, build_depends=True, test_depends=False)
        print "Found wet build dependencies:\n%s" % '- ' + '\n- '.join(sorted(repo_build_dependencies))
    except:
        pass
    # all packages in sourcespace
    (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace)
    print catkin_packages
    print stacks
    print manifest_packages
    # deps catkin
    repo_build_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
    print "Found wet dependencies:\n%s" % '- ' + '\n- '.join(sorted(repo_build_dependencies))
    # deps stacks
    repo_build_dependencies = cob_common.get_nonlocal_dependencies({}, stacks, {})
    print "Found dry dependencies:\n%s" % '- ' + '\n- '.join(sorted(repo_build_dependencies))

    # check if build_repo is wet or dry and take corresponding deps
    if build_repo in catkin_packages:
        catkin_build_repo = True
        repo_build_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
        #repo_build_dependencies = cob_common.get_dependencies(repo_sourcespace, build_depends=True, test_depends=False)
    elif build_repo in stacks:
        catkin_build_repo = False
        repo_build_dependencies = cob_common.get_nonlocal_dependencies({}, stacks, {})
    else:
        # build_repo is neither wet nor dry
        raise cob_common.BuildException("Repository to build not found in sourcespace")

    # install user-defined/customized dependencies from source
    rosinstall = ''
    fulfilled_deps = []
    for dep in repo_build_dependencies:
        if dep in buildpipe_repos[build_repo].dependencies:
            print "Install user-defined build dependency %s from source" % dep
            rosinstall += buildpipe_repos[build_repo].dependencies[dep].get_rosinstall()
            fulfilled_deps.append(dep)

    # check if all user-defined/customized dependencies are satisfied
    if sorted(fulfilled_deps) != sorted(buildpipe_repos[build_repo].dependencies):
        print "Not all user-defined build dependencies are fulfilled"
        print "User-defined build dependencies:\n%s" % '- ' + '\n- '.join(buildpipe_repos[build_repo].dependencies)
        print "Fulfilled dependencies:\n%s" % '- ' + '\n- '.join(fulfilled_deps)
        raise cob_common.BuildException("Not all user-defined build dependencies are fulfilled")

    if rosinstall != '':
        # write .rosinstall file
        print "Rosinstall file for user-defined build dependencies: \n %s" % rosinstall
        with open(os.path.join(workspace, "repo.rosinstall"), 'w') as f:
            f.write(rosinstall)
        print "Install user-defined build dependencies from source"
        # rosinstall depends
        # TODO handle dry stacks
        cob_common.call("rosinstall %s %s/repo.rosinstall --catkin"
                        % (repo_sourcespace, workspace))

        # get all deps also of just installed user-defined/customized dependencies
        (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace)
        if catkin_build_repo:
            if stacks != {}:
                raise cob_common.BuildException("Catkin (wet) package %s depends on (dry) stack(s):\n%s"
                                                % (build_repo, '- ' + '\n- '.join(stacks)))
            # take only wet packages
            repo_build_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
            #repo_build_dependencies = cob_common.get_dependencies(repo_sourcespace, build_depends=True, test_depends=False)
        else:  # dry build repo
            # take all packages
            repo_build_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, stacks, {}, build_depends=True, test_depends=False)
        repo_build_dependencies = [dep for dep in repo_build_dependencies if dep not in fulfilled_deps]

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
    # TODO
    (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace)
    if catkin_build_repo:
        if stacks != {}:
            raise cob_common.BuildException("Catkin (wet) package %s depends on (dry) stack(s):\n%s"
                                            % (build_repo, '- ' + '\n- '.join(stacks)))
        # take only wet packages
        repo_test_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=False, test_depends=True)
    else:  # dry build repo
        # take all packages
        repo_test_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, stacks, {}, build_depends=False, test_depends=True)
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
