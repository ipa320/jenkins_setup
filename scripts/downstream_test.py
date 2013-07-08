#!/usr/bin/env python

import optparse
import os
import sys
import shutil
import rosdistro

from jenkins_setup import common, cob_pipe, rosdep


def main():
    # parse parameter values
    parser = optparse.OptionParser()
    parser.add_option('-v', '--verbose', action='store_true', default=False)
    (options, args) = parser.parse_args()

    if len(args) < 4:
        print "Usage: %s server_name user_name ros_distro build_repo" % sys.argv[0]
        raise common.BuildException("Wrong arguments for build script")

    # get arguments
    server_name = args[0]
    #user_name = args[1]
    ros_distro = args[2]
    build_identifier = args[3]
    build_repo = build_identifier.split('__')[0]  # repository to build
    workspace = os.environ['WORKSPACE']
    ros_package_path = os.environ['ROS_PACKAGE_PATH']

    # (debug) output
    print "\n", 50 * 'X'
    print "\nTesting on ros distro:  %s" % ros_distro
    print "Testing repository: %s" % build_repo.split('__')[0]
    if len(build_repo.split('__')) > 1:
        print "       with suffix: %s" % '__'.join(build_repo.split('__')[1:])
    print "\n", 50 * 'X'

    if ros_distro == 'electric':
        pass
    else:
        build_downstream_post_fuerte(ros_distro, build_repo, workspace, server_name)


def build_downstream_post_fuerte(ros_distro, build_repo, workspace, server):
    ros_package_path = os.environ['ROS_PACKAGE_PATH']
    b_r_short = build_repo.split('__')[0]

    # set up directories variables
    tmpdir = os.path.join('/tmp', 'test_repositories')
    repo_sourcespace = os.path.join(tmpdir, 'src_repository')
    repo_sourcespace_dry = os.path.join(tmpdir, 'src_repository', 'dry')
    repo_buildspace = os.path.join(tmpdir, 'build_repository')
    dependson_sourcespace = os.path.join(tmpdir, 'src_depends_on')
    dependson_sourcespace_wet = os.path.join(tmpdir, 'src_depends_on', 'wet')
    dependson_sourcespace_dry = os.path.join(tmpdir, 'src_depends_on', 'dry')
    dependson_buildspace = os.path.join(tmpdir, 'build_depend_on')
    dry_test_results_dir = os.path.join(dependson_sourcespace_dry, 'test_results')

    # TODO clean up test result dirs

    # get depends on
    print "Get list of released repositories that (build-)depend on repository %s" % b_r_short
    ros_depends_on = []
    distro_ros = rosdistro.RosDistro(ros_distro, 'http://%s/~jenkins' % server)
    for d in distro_ros.get_depends_on([b_r_short])['build'] + distro_ros.get_depends_on([b_r_short])['buildtool']:
        if not d in ros_depends_on and d != b_r_short:
            ros_depends_on.append(d)
    # TODO get_depends_on of cob_distro release (only for intern repos necessary)
    cob_depends_on = []
    #distro_cob = cob_distro.CobDistro(ros_distro)
    #for d in distro_cob.get_depends_on([b_r_short])['build'] + distro_cob.get_depends_on([b_r_short])['buildtool']:
    #    if not d in cob_depends_on and d != b_r_short:
    #        cob_depends_on.append(d)
    if len(ros_depends_on + cob_depends_on) == 0:
        print "No repository depends on repository %s. Test finished" % b_r_short
        return

    print "Build depends_on list of repository %s:\n - %s" % ('\n - '.join(ros_depends_on + cob_depends_on))

    # install depends_on repository from source
    rosinstall = distro_ros.get_rosinstall(ros_depends_on)
    #rosinstall += distro_cob.get_rosinstall(cob_depends_on) TODO
    print "Rosinstall for depends_on:\n %s" % rosinstall
    with open(workspace + "/depends_on.rosinstall", 'w') as f:
        f.write(rosinstall)
    print "Created rosinstall file for depends on"

    # install all repository and system dependencies of the depends_on list
    print "Install all depends_on from source: %s" % (', '.join(ros_depends_on))
    os.makedirs(dependson_sourcespace)
    common.call("rosinstall %s %s/depends_on.rosinstall /opt/ros/%s" % (dependson_sourcespace, workspace, ros_distro))

    # all packages in dependson_sourcespace
    (catkin_packages, stacks, manifest_packages) = common.get_all_packages(dependson_sourcespace)
    print catkin_packages
    print stacks
    print manifest_packages

    # get build and test dependencies of depends_on list
    dependson_build_dependencies = []
    for d in common.get_nonlocal_dependencies(catkin_packages, stacks, {}, build_depends=True, test_depends=False):
        print "  Checking dependency %s" % d
        if d in dependson_build_dependencies:
            print "    Already in dependson_build_dependencies"
        elif d in ros_depends_on or d in cob_depends_on:
            print "    Is a direct dependency of the repository, and is installed from source"
        elif d == b_r_short:
            print "    Is the tested repository"
        else:
            dependson_build_dependencies.append(d)
    print "Build dependencies of depends_on list are %s" % (', '.join(dependson_build_dependencies))

    dependson_test_dependencies = []
    for d in common.get_nonlocal_dependencies(catkin_packages, stacks, {}, build_depends=False, test_depends=True):
        if d not in dependson_test_dependencies + ros_depends_on + cob_depends_on and d != b_r_short:
            dependson_test_dependencies.append(d)
    print "Test dependencies of depends_on list are %s" % (', '.join(dependson_test_dependencies))

    # separate installed repos in wet and dry
    print "Separate installed repositories in wet and dry"
    os.makedirs(dependson_sourcespace_wet)
    os.makedirs(dependson_sourcespace_dry)
    # get all folders in dependson_sourcespace
    sourcespace_dirs = [name for name in os.listdir(dependson_sourcespace) if os.path.isdir(os.path.join(dependson_sourcespace, name))]
    for dir in sourcespace_dirs:
        if dir in catkin_packages.keys():
            shutil.move(os.path.join(dependson_sourcespace, dir), os.path.join(dependson_sourcespace_wet, dir))
        if dir in stacks.keys():
            shutil.move(os.path.join(dependson_sourcespace, dir), os.path.join(dependson_sourcespace_dry, dir))

    # Create rosdep object
    print "Create rosdep object"
    try:
        rosdep_resolver = rosdep.RosDepResolver(ros_distro)
    except:
        from time import sleep
        sleep(10)
        rosdep_resolver = rosdep.RosDepResolver(ros_distro)

    # install build dependencies
    print "Install all build dependencies of the depends_on list: %s" % (', '.join(dependson_build_dependencies))
    common.apt_get_install_also_nonrosdep(dependson_build_dependencies, rosdep_resolver)

    # env
    print "Setting up environment"
    ros_env = common.get_ros_env('/opt/ros/%s/setup.bash' % ros_distro)
    common.call("env", ros_env)
    ros_env_dependson = common.get_ros_env(os.path.join(repo_sourcespace, 'setup.bash'))
    ros_env_dependson['ROS_PACKAGE_PATH'] = ':'.join([dependson_buildspace, repo_buildspace, ros_package_path])
    common.call("env", ros_env_dependson)

    ### catkin repositories
    if catkin_packages != {}:
        os.makedirs(dependson_buildspace)
        os.chdir(dependson_buildspace)
        print "Create a new CMakeLists.txt for catkin packages"
        if ros_distro == 'fuerte':
            common.call("ln -s %s %s" % (os.path.join(dependson_sourcespace_wet, 'catkin', 'cmake', 'toplevel.cmake'),
                                         os.path.join(dependson_sourcespace_wet, 'CMakeLists.txt')))
        else:
            common.call("catkin_init_workspace %s" % dependson_sourcespace_wet, ros_env_dependson)

        try:
            common.call("cmake %s" % dependson_sourcespace_wet, ros_env_dependson)
        except common.BuildException as ex:
            print ex.msg
            raise common.BuildException("Failed to cmake wet repositories")
        #ros_env_repo = common.get_ros_env(os.path.join(repo_buildspace, 'devel/setup.bash'))

        # build repositories
        print "Build wet depends_on list"
        try:
            common.call("make", ros_env_dependson)
        except common.BuildException as ex:
            print ex.msg
            raise common.BuildException("Failed to make wet packages")

        if dependson_test_dependencies != []:
            # install test dependencies
            print "Install all test dependencies of the depends_on list: %s" % (', '.join(dependson_test_dependencies))
            common.apt_get_install_also_nonrosdep(dependson_test_dependencies, rosdep_resolver)

            # test repositories
            try:
                common.call("make run_tests", ros_env_dependson)
            except common.BuildException as ex:
                print ex.msg

        # copy test results
        common.copy_test_results(workspace, dependson_buildspace)

    ### rosbuild repositories
    if stacks != {}:
        # env
        print "Setting up environment"
        ros_env_dependson['ROS_PACKAGE_PATH'] = ':'.join([dependson_buildspace, repo_buildspace,
                                                          dependson_sourcespace_dry, repo_sourcespace_dry,
                                                          ros_package_path])
        common.call("env", ros_env_dependson)

        #print "Make rosdep"
        #common.call("rosmake rosdep", ros_env)
        #for stack in stacks.keys():
        #    common.call("rosdep install -y %s" % stack, ros_env_repo)

        dependson_sourcespace_dry_dirs = [name for name in os.listdir(dependson_sourcespace_dry) if os.path.isdir(os.path.join(dependson_sourcespace_dry, name))]
        print "Build dry depends_on repositories:\n - %s" % '\n - '.join(dependson_sourcespace_dry_dirs)
        os.mkdir(dry_test_results_dir)
        for dry_dependson in dependson_sourcespace_dry_dirs:
            try:
                common.call("rosmake -rV --profile --pjobs=8 --output=%s %s" % (dry_test_results_dir, b_r_short), ros_env_dependson)
            except:
                raise common.BuildException("Failed to rosmake %s" % b_r_short)
            try:
                common.call("rosmake -rV --profile --pjobs=8 --test-only --output=%s %s" % (dry_test_results_dir, b_r_short), ros_env_dependson)
                # TODO output dir ??
            except:
                print "Failed to test %s" % dry_dependson
        # copy test results
        common.call("rosrun rosunit clean_junit_xml.py", ros_env_dependson)
        common.copy_test_results(workspace, dependson_sourcespace_dry)


if __name__ == "__main__":
    # global try
    try:
        main()
        print "Build script finished cleanly!"

    # global catch
    except (common.BuildException, cob_pipe.CobPipeException) as ex:
        print "Build script failed!"
        print ex.msg
        raise ex

    except Exception as ex:
        print "Build script failed! Check out the console output above for details."
        raise ex
