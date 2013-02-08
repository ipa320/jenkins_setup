#!/usr/bin/env python

import optparse
import sys
import os
import shutil

from jenkins_setup import cob_common, rosdep, cob_pipe


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
    print "\nTesting on ros distro:  %s" % ros_distro
    print "Testing the repository: %s" % build_repo.split('__')[0]
    if len(build_repo.split('__')) > 1:
        print "           with suffix: %s" % '__'.join(build_repo.split('__')[1:])
    print "\n", 50 * 'X'

    # update sourcelist and upgrade installed basic packages
    print "\nUpdating chroot enviroment installed packages"
    cob_common.call("apt-get update")
    cob_common.call("apt-get dist-upgrade -y")

    print "\nUpdating rosinstall"  # TODO run install frequently in chroot_tarball_updater an remove here
    cob_common.call("pip install -U rosinstall")

    # cob_pipe object
    cp_instance = cob_pipe.Cob_Pipe()
    cp_instance.load_config_from_url(server_name, user_name)
    buildpipe_repos = cp_instance.repositories
    print "Pipeline configuration successfully loaded"

    # build depending on ros_ distro
    if ros_distro == 'electric':
        build_electric(ros_distro, build_repo, buildpipe_repos, workspace)
    elif ros_distro == 'fuerte':
        build_fuerte(ros_distro, build_repo, buildpipe_repos, workspace)
    else:
        build_post_fuerte(ros_distro, build_repo, buildpipe_repos, workspace)


def build_electric(ros_distro, build_repo, buildpipe_repos, workspace):
    ros_package_path = os.environ['ROS_PACKAGE_PATH']
    b_r_short = build_repo.split('__')[0]

    # set up directories variables
    tmpdir = os.path.join('/tmp', 'test_repositories')
    repo_sourcespace = os.path.join(tmpdir, 'src_repository')
    test_results_dir = os.path.join(repo_sourcespace, 'test_results')

    # install Debian packages needed for script TODO ??
    #print "Installing Debian packages we need for running this script"
    #cob_common.call("apt-get install python-catkin-pkg python-rosdistro --yes")
    print "\nUpdating catkin-pkg and rospkg"
    cob_common.call("pip install -U catkin-pkg rospkg")

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
    ros_env_repo['ROS_PACKAGE_PATH'] = ':'.join([repo_sourcespace, ros_package_path])
    cob_common.call("env", ros_env_repo)
    print "Rosdep"
    cob_common.call("rosmake rosdep", ros_env)
    for stack in stacks.keys():
        cob_common.call("rosdep install -y %s" % stack, ros_env_repo)

    # TODO build (like in hudson_helper)
    # build repositories and tests
    print "Build repository %s" % b_r_short
    try:
        cob_common.call("rosmake -rV --profile --pjobs=8 --output=%s %s" % (test_results_dir, b_r_short), ros_env_repo)
    except:
        raise cob_common.BuildException("Failed to rosmake %s" % b_r_short)
    try:
        cob_common.call("rosmake -rV --profile --pjobs=8 --test-only --output=%s %s" % (test_results_dir, b_r_short), ros_env_repo)
        # TODO output dir ??
    finally:
        # copy test results
        cob_common.call("rosrun rosunit clean_junit_xml.py", ros_env)
        cob_common.copy_test_results(workspace, repo_sourcespace)


def build_fuerte(ros_distro, build_repo, buildpipe_repos, workspace):
    ros_package_path = os.environ['ROS_PACKAGE_PATH']
    b_r_short = build_repo.split('__')[0]

    # set up directories variables
    tmpdir = os.path.join('/tmp', 'test_repositories')
    repo_sourcespace = os.path.join(tmpdir, 'src_repository')
    repo_sourcespace_wet = os.path.join(tmpdir, 'src_repository', 'wet')
    repo_sourcespace_dry = os.path.join(tmpdir, 'src_repository', 'dry')
    repo_buildspace = os.path.join(tmpdir, 'build_repository')
    dry_test_results_dir = os.path.join(repo_sourcespace_dry, 'test_results')

    # install Debian packages needed for script
    print "Installing Debian packages we need for running this script"
    cob_common.call("pip install -U rospkg rosdep")

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
    cob_common.call("rosinstall %s %s/repo.rosinstall /opt/ros/%s"
                    % (repo_sourcespace, workspace, ros_distro))

    # get the repositories build dependencies
    print "Get build dependencies of repo"

    # all packages in sourcespace
    (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace)
    print catkin_packages
    print stacks
    print manifest_packages

    ### DEBUG ###
    # get deps directly for catkin like willow
    try:
        repo_build_dependencies = cob_common.get_dependencies(repo_sourcespace, build_depends=True, test_depends=False)
        print "Found wet build dependencies:\n%s" % '- ' + '\n- '.join(sorted(repo_build_dependencies))
    except:
        pass
    # deps catkin
    repo_build_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
    print "Found wet dependencies:\n%s" % '- ' + '\n- '.join(sorted(repo_build_dependencies))
    # deps stacks
    repo_build_dependencies = cob_common.get_nonlocal_dependencies({}, stacks, {})
    print "Found dry dependencies:\n%s" % '- ' + '\n- '.join(sorted(repo_build_dependencies))
    ###############

    # check if build_repo is wet or dry and take corresponding deps
    build_repo_type = ''
    if b_r_short in catkin_packages:
        build_repo_type = 'wet'
        repo_build_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
    elif b_r_short in stacks:
        build_repo_type = 'dry'
        repo_build_dependencies = cob_common.get_nonlocal_dependencies({}, stacks, {})
    else:
        # b_r_short is neither wet nor dry
        raise cob_common.BuildException("Repository %s to build not found in sourcespace" % b_r_short)

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

        # get all deps also of just installed user-defined/customized dependencies
        (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace)
        if build_repo_type == 'wet':
            if stacks != {}:
                raise cob_common.BuildException("Catkin (wet) package %s depends on (dry) stack(s):\n%s"
                                                % (b_r_short, '- ' + '\n- '.join(stacks)))
            # take only wet packages
            repo_build_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
        else:  # dry build repo
            # take all packages
            repo_build_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, stacks, {}, build_depends=True, test_depends=False)
        repo_build_dependencies = [dep for dep in repo_build_dependencies if dep not in fulfilled_deps]

    # Create rosdep object
    print "Create rosdep object"
    try:
        rosdep_resolver = rosdep.RosDepResolver(ros_distro)
    except:
        from time import sleep
        sleep(10)
        rosdep_resolver = rosdep.RosDepResolver(ros_distro)

    print "Install build dependencies of repo list: %s" % (', '.join(repo_build_dependencies))
    repo_build_dependencies_rosdep = []
    repo_build_dependencies_aptget = []
    for repo_build_dep in repo_build_dependencies:
        if rosdep_resolver.has_ros(repo_build_dep):
            repo_build_dependencies_rosdep.append(repo_build_dep)
        else:
            repo_build_dependencies_aptget.append('-'.join(['ros', ros_distro, repo_build_dep.replace('_', '-')]))
    cob_common.apt_get_install(repo_build_dependencies_rosdep, rosdep_resolver)
    if repo_build_dependencies_aptget != []:
        print "The following dependencies couldn't be found in the rosdep database: \n - %s" % '\n - '.join(repo_build_dependencies_aptget)
        print "Trying to install them via 'ros-%s-<dependenpy_name>'" % ros_distro
        try:
            cob_common.apt_get_install(repo_build_dependencies_aptget)
        except:
            raise cob_common.BuildException("A dependency could neighter be found in the rosdep database nor installed directly")

    # separate installed repos in wet and dry
    print "Separate installed repositories in wet and dry"
    os.makedirs(repo_sourcespace_wet)
    os.makedirs(repo_sourcespace_dry)
    # get all folders in repo_sourcespace
    sourcespace_dirs = [name for name in os.listdir(repo_sourcespace) if os.path.isdir(os.path.join(repo_sourcespace, name))]
    for dir in sourcespace_dirs:
        if dir in catkin_packages.keys():
            shutil.move(os.path.join(repo_sourcespace, dir), os.path.join(repo_sourcespace_wet, dir))
        if dir in stacks.keys():
            shutil.move(os.path.join(repo_sourcespace, dir), os.path.join(repo_sourcespace_dry, dir))
    shutil.move(os.path.join(repo_sourcespace, 'setup.sh'), os.path.join(repo_sourcespace_dry, 'setup.sh'))
    shutil.move(os.path.join(repo_sourcespace, 'setup.bash'), os.path.join(repo_sourcespace_dry, 'setup.bash'))
    print cob_common.get_all_packages(repo_sourcespace)

    # env
    print "Set up environment variables"
    ros_env = cob_common.get_ros_env('/opt/ros/%s/setup.bash' % ros_distro)

    ### catkin repositories
    # add catkin package to rosinstall
    if catkin_packages != {}:
        if 'catkin' not in catkin_packages.keys():
            rosinstall = "\n- git: {local-name: catkin, uri: 'git://github.com/ros/catkin.git', version: fuerte-devel}"
            print "Install catkin"
            # rosinstall catkin
            cob_common.call("rosinstall %s %s/repo.rosinstall /opt/ros/%s"
                            % (repo_sourcespace_wet, workspace, ros_distro))

        print "Create a new CMakeLists.txt for catkin packages"
        cob_common.call("ln -s %s %s" % (os.path.join(repo_sourcespace_wet, 'catkin', 'cmake', 'toplevel.cmake'),
                                         os.path.join(repo_sourcespace_wet, 'CMakeLists.txt')))
        os.mkdir(repo_buildspace)
        os.chdir(repo_buildspace)
        cob_common.call("cmake %s" % repo_sourcespace_wet + '/', ros_env)

        # build repositories and tests
        print "Build wet repo list"
        cob_common.call("make", ros_env)
        cob_common.call("make tests", ros_env)

        # get wet repositories test and run dependencies
        print "Get test and run dependencies of repo list"
        (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace_wet)
        if stacks != {}:
            raise cob_common.BuildException("Catkin (wet) package %s depends on (dry) stack(s):\n%s"
                                            % (b_r_short, '- ' + '\n- '.join(stacks)))
        # take only wet packages
        repo_test_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=False, test_depends=True)
        if repo_test_dependencies != []:
            print "Install test and run dependencies of repo list: %s" % (', '.join(repo_test_dependencies))
            cob_common.apt_get_install(repo_test_dependencies, rosdep_resolver)

            # run tests
            print "Test repo list"
            cob_common.call("make run_tests", ros_env)

            # copy test results
            cob_common.copy_test_results(workspace, repo_buildspace)

    ### rosbuild repositories
    ros_env_repo = cob_common.get_ros_env(os.path.join(repo_sourcespace_dry, 'setup.bash'))
    ros_env_repo['ROS_PACKAGE_PATH'] = ':'.join([repo_buildspace, repo_sourcespace, ros_package_path])

    if build_repo_type == 'dry':
        #print "Make rosdep"
        #cob_common.call("rosmake rosdep", ros_env)
        #for stack in stacks.keys():
        #    cob_common.call("rosdep install -y %s" % stack, ros_env_repo)

        print "Build dry repo list"
        os.mkdir(dry_test_results_dir)
        cob_common.call("rosmake -rV --profile --pjobs=8 --output=%s %s" % (dry_test_results_dir, b_r_short), ros_env_repo)
        cob_common.call("rosmake -rV --profile --pjobs=8 --test-only --output=%s %s" % (dry_test_results_dir, b_r_short), ros_env_repo)

        # copy test results
        cob_common.call("rosrun rosunit clean_junit_xml.py", ros_env)
        cob_common.copy_test_results(workspace, repo_sourcespace_dry)


def build_post_fuerte(ros_distro, build_repo, buildpipe_repos, workspace):
    ros_package_path = os.environ['ROS_PACKAGE_PATH']
    b_r_short = build_repo.split('__')[0]

    # set up directories variables
    tmpdir = os.path.join('/tmp', 'test_repositories')
    repo_sourcespace = os.path.join(tmpdir, 'src_repository')
    repo_sourcespace_wet = os.path.join(tmpdir, 'src_repository', 'wet')
    repo_sourcespace_dry = os.path.join(tmpdir, 'src_repository', 'dry')
    repo_buildspace = os.path.join(tmpdir, 'build_repository')
    dry_test_results_dir = os.path.join(repo_sourcespace_dry, 'test_results')

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
    cob_common.call("rosinstall %s %s/repo.rosinstall /opt/ros/%s"
                    % (repo_sourcespace, workspace, ros_distro))

    # get the repositories build dependencies
    print "Get build dependencies of repo"

    # all packages in sourcespace
    (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace)
    print catkin_packages
    print stacks
    print manifest_packages

    ### DEBUG ###
    # get deps directly for catkin like willow
    try:
        repo_build_dependencies = cob_common.get_dependencies(repo_sourcespace, build_depends=True, test_depends=False)
        print "Found wet build dependencies:\n%s" % '- ' + '\n- '.join(sorted(repo_build_dependencies))
    except:
        pass
    # deps catkin
    repo_build_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
    print "Found wet dependencies:\n%s" % '- ' + '\n- '.join(sorted(repo_build_dependencies))
    # deps stacks
    repo_build_dependencies = cob_common.get_nonlocal_dependencies({}, stacks, {})
    print "Found dry dependencies:\n%s" % '- ' + '\n- '.join(sorted(repo_build_dependencies))
    ###############

    # check if build_repo is wet or dry and take corresponding deps
    build_repo_type = ''
    if b_r_short in catkin_packages:
        build_repo_type = 'wet'
        repo_build_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
    elif b_r_short in stacks:
        build_repo_type = 'dry'
        repo_build_dependencies = cob_common.get_nonlocal_dependencies({}, stacks, {})
    else:
        # b_r_short is neither wet nor dry
        raise cob_common.BuildException("Repository %s to build not found in sourcespace" % b_r_short)

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

        # get all deps also of just installed user-defined/customized dependencies
        (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace)
        if build_repo_type == 'wet':
            if stacks != {}:
                raise cob_common.BuildException("Catkin (wet) package %s depends on (dry) stack(s):\n%s"
                                                % (b_r_short, '- ' + '\n- '.join(stacks)))
            # take only wet packages
            repo_build_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
        else:  # dry build repo
            # take all packages
            repo_build_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, stacks, {}, build_depends=True, test_depends=False)
        repo_build_dependencies = [dep for dep in repo_build_dependencies if dep not in fulfilled_deps]

    # Create rosdep object
    print "Create rosdep object"
    try:
        rosdep_resolver = rosdep.RosDepResolver(ros_distro)
    except:
        from time import sleep
        sleep(10)
        rosdep_resolver = rosdep.RosDepResolver(ros_distro)

    print "Install build dependencies of repo list: %s" % (', '.join(repo_build_dependencies))
    repo_build_dependencies_rosdep = []
    repo_build_dependencies_aptget = []
    for repo_build_dep in repo_build_dependencies:
        if rosdep_resolver.has_ros(repo_build_dep):
            repo_build_dependencies_rosdep.append(repo_build_dep)
        else:
            repo_build_dependencies_aptget.append('-'.join(['ros', ros_distro, repo_build_dep.replace('_', '-')]))
    cob_common.apt_get_install(repo_build_dependencies_rosdep, rosdep_resolver)
    if repo_build_dependencies_aptget != []:
        print "The following dependencies couldn't be found in the rosdep database: \n - %s" % '\n - '.join(repo_build_dependencies_aptget)
        print "Trying to install them via 'ros-%s-<dependenpy_name>'" % ros_distro
        try:
            cob_common.apt_get_install(repo_build_dependencies_aptget)
        except:
            raise cob_common.BuildException("A dependency could neighter be found in the rosdep database nor installed directly")

    # separate installed repos in wet and dry
    print "Separate installed repositories in wet and dry"
    os.makedirs(repo_sourcespace_wet)
    os.makedirs(repo_sourcespace_dry)
    # get all folders in repo_sourcespace
    sourcespace_dirs = [name for name in os.listdir(repo_sourcespace) if os.path.isdir(os.path.join(repo_sourcespace, name))]
    for dir in sourcespace_dirs:
        if dir in catkin_packages.keys():
            shutil.move(os.path.join(repo_sourcespace, dir), os.path.join(repo_sourcespace_wet, dir))
        if dir in stacks.keys():
            shutil.move(os.path.join(repo_sourcespace, dir), os.path.join(repo_sourcespace_dry, dir))
    shutil.move(os.path.join(repo_sourcespace, 'setup.sh'), os.path.join(repo_sourcespace_dry, 'setup.sh'))
    shutil.move(os.path.join(repo_sourcespace, 'setup.bash'), os.path.join(repo_sourcespace_dry, 'setup.bash'))

    # env
    print "Set up environment variables"
    ros_env = cob_common.get_ros_env('/opt/ros/%s/setup.bash' % ros_distro)

    ### catkin repositories
    if catkin_packages != {}:
        print "Create a CMakeLists.txt for catkin packages"
        cob_common.call("catkin_init_workspace %s" % repo_sourcespace_wet, ros_env)

        os.makedirs(repo_buildspace)
        os.chdir(repo_buildspace)
        cob_common.call("cmake %s" % repo_sourcespace_wet, ros_env)
        #ros_env_repo = cob_common.get_ros_env(os.path.join(repo_buildspace, 'devel/setup.bash'))

        # build repositories and tests
        print "Build repo list"
        cob_common.call("make", ros_env)
        cob_common.call("make tests", ros_env)

        # get the repositories test and run dependencies
        print "Get test and run dependencies of repo list"
        (catkin_packages, stacks, manifest_packages) = cob_common.get_all_packages(repo_sourcespace_wet)
        if stacks != {}:
            raise cob_common.BuildException("Catkin (wet) package %s depends on (dry) stack(s):\n%s"
                                            % (b_r_short, '- ' + '\n- '.join(stacks)))
        # take only wet packages
        repo_test_dependencies = cob_common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=False, test_depends=True)
        if repo_test_dependencies != []:
            print "Install test and run dependencies of repo list: %s" % (', '.join(repo_test_dependencies))
            cob_common.apt_get_install(repo_test_dependencies, rosdep_resolver)

            # run tests
            print "Test repo list"
            cob_common.call("make run_tests", ros_env)

            # copy test results
            cob_common.copy_test_results(workspace, repo_buildspace)

    ### rosbuild repositories
    ros_env_repo = cob_common.get_ros_env(os.path.join(repo_sourcespace_dry, 'setup.bash'))
    ros_env_repo['ROS_PACKAGE_PATH'] = ':'.join([repo_buildspace, repo_sourcespace, ros_package_path])

    if build_repo_type == 'dry':
        #print "Make rosdep"
        #cob_common.call("rosmake rosdep", ros_env)
        #for stack in stacks.keys():
        #    cob_common.call("rosdep install -y %s" % stack, ros_env_repo)

        print "Build dry repo list"
        os.mkdir(dry_test_results_dir)
        cob_common.call("rosmake -rV --profile --pjobs=8 --output=%s %s" % (dry_test_results_dir, b_r_short), ros_env_repo)
        cob_common.call("rosmake -rV --profile --pjobs=8 --test-only --output=%s %s" % (dry_test_results_dir, b_r_short), ros_env_repo)

        # copy test results
        cob_common.call("rosrun rosunit clean_junit_xml.py", ros_env)
        cob_common.copy_test_results(workspace, repo_sourcespace_dry)


if __name__ == "__main__":
    # global try
    try:
        main()
        print "Build script finished cleanly!"

    # global catch
    except (cob_common.BuildException, cob_pipe.CobPipeException) as ex:
        print "Build script failed!"
        print ex.msg
        raise ex

    except Exception as ex:
        print "Build script failed! Check out the console output above for details."
        raise ex
