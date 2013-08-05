#!/usr/bin/env python

import optparse
import sys
import os
import shutil
import datetime

from jenkins_setup import common, rosdep, cob_pipe


def main():
    # parse parameter values
    parser = optparse.OptionParser()
    parser.add_option('-v', '--verbose', action='store_true', default=False)
    (options, args) = parser.parse_args()

    if len(args) < 6:
        print "Usage: %s pipeline_repos_owner server_name user_name ros_distro build_repo graphic_test" % sys.argv[0]
        raise common.BuildException("Wrong arguments for build script")

    # get arguments
    pipeline_repos_owner = args[0]
    server_name = args[1]
    user_name = args[2]
    ros_distro = args[3]
    build_identifier = args[4]                      # repository + suffix
    build_repo = build_identifier.split('__')[0]    # only repository to build
    graphic_test = True if args[5] == "true" else False  # TODO optional
    build_repo_only = True if args[6] == "true" else False
    # environment variables
    workspace = os.environ['WORKSPACE']
    ros_package_path = os.environ['ROS_PACKAGE_PATH']

    # cob_pipe object
    cp_instance = cob_pipe.CobPipe()
    cp_instance.load_config_from_url(pipeline_repos_owner, server_name, user_name)
    pipe_repos = cp_instance.repositories
    common.output("Pipeline configuration successfully loaded", blankline='b')

    # (debug) output
    print "\n", 50 * 'X'
    print datetime.datetime.now()
    print "\nTesting on ros distro:  %s" % ros_distro
    print "Testing repository: %s" % build_repo
    print "Graphic Test: %s" % graphic_test
    print "Build Repo Only: %s" % build_repo_only
    if build_repo != build_identifier:
        print "       with suffix: %s" % build_identifier.split('__')[1]
    print "Using source: %s" % pipe_repos[build_identifier].url
    print "Testing branch/version: %s" % pipe_repos[build_identifier].version
    print "\n", 50 * 'X'

    # set up directories variables
    tmpdir = os.path.join('/tmp', 'test_repositories')
    repo_sourcespace = os.path.join(tmpdir, 'src_repository')               # location to store repositories in
    repo_sourcespace_wet = os.path.join(tmpdir, 'src_repository', 'wet')    # wet (catkin) repositories
    repo_sourcespace_dry = os.path.join(tmpdir, 'src_repository', 'dry')    # dry (rosbuild) repositories
    repo_buildspace = os.path.join(tmpdir, 'build_repository')              # location for build output
    dry_build_logs = os.path.join(repo_sourcespace_dry, 'build_logs')       # location for build logs

    if ros_distro != 'electric':
        # Create rosdep object
        print "Create rosdep object"
        try:
            rosdep_resolver = rosdep.RosDepResolver(ros_distro)
        except:  # when init fails the first time
            from time import sleep
            sleep(10)
            rosdep_resolver = rosdep.RosDepResolver(ros_distro)

    # env
    print "Set up ros environment variables"
    ros_env = common.get_ros_env('/opt/ros/%s/setup.bash' % ros_distro)
    if options.verbose:
        common.call("env", ros_env)

    ### catkin repositories
    print datetime.datetime.now()

    if os.listdir(repo_sourcespace_wet):
        # set up catkin workspace
        #if ros_distro == 'fuerte':
            #if 'catkin' not in catkin_packages.keys():
                ## add catkin package to rosinstall
                #rosinstall = "\n- git: {local-name: catkin, uri: 'git://github.com/ros/catkin.git', version: fuerte-devel}"
                #print "Install catkin"
                ## rosinstall catkin
                #common.call("rosinstall --verbose %s %s/repo.rosinstall /opt/ros/%s"
                            #% (repo_sourcespace_wet, workspace, ros_distro))

            #print "Create a CMakeLists.txt for catkin packages"
            #common.call("ln -s %s %s" % (os.path.join(repo_sourcespace_wet, 'catkin', 'cmake', 'toplevel.cmake'),
                                         #os.path.join(repo_sourcespace_wet, 'CMakeLists.txt')))
        #else:
            #common.call("catkin_init_workspace %s" % repo_sourcespace_wet, ros_env)

        os.mkdir(repo_buildspace)
        os.chdir(repo_buildspace)

        # test repositories
        print "Test wet repository list"
        test_error_msg = None
        try:
            common.call("make tests", ros_env)
        except common.BuildException as ex:
            print ex.msg
            test_error_msg = ex.msg

        # get wet repositories test and run dependencies
        print "Get test and run dependencies of repo list"
        (catkin_packages, stacks, manifest_packages) = common.get_all_packages(repo_sourcespace_wet)
        if stacks != {}:
            raise common.BuildException("Catkin (wet) package %s depends on (dry) stack(s):\n%s"
                                        % (build_repo, '- ' + '\n- '.join(stacks)))
        # take only wet packages
        repo_test_dependencies = common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=False, test_depends=True)
        if repo_test_dependencies != [] and test_error_msg is None:
            print "Install test and run dependencies of repository list: %s" % (', '.join(repo_test_dependencies))
            common.apt_get_install_also_nonrosdep(repo_test_dependencies, ros_distro, rosdep_resolver)

            # run tests
            print "Test repository list"
            try:
                common.call("%s make run_tests" % "/opt/VirtualGL/bin/vglrun" if graphic_test else "", ros_env)  # TODO check how to test a list of repos
            except common.BuildException as ex:
                print ex.msg
                test_error_msg = ex.msg

        # copy test results
        common.copy_test_results(workspace, repo_buildspace, test_error_msg)

    ### rosbuild repositories
    print datetime.datetime.now()
    (catkin_packages, stacks, manifest_packages) = common.get_all_packages(repo_sourcespace_dry)
    if build_repo in stacks:
        # get list of dependencies to test
        test_repos_list = []
        for dep in pipe_repos[build_identifier].dependencies:
            if dep in stacks:  # TODO option to select deps to build
                test_repos_list.append(dep)

        ros_env_repo = common.get_ros_env(os.path.join(repo_sourcespace, 'setup.bash'))
        ros_env_repo['ROS_PACKAGE_PATH'] = ':'.join([repo_sourcespace, ros_package_path])
        if options.verbose:
            common.call("env", ros_env_repo)

        # test dry repositories
        print "Test repository %s" % build_repo
        try:
            build_list = " ".join(test_repos_list + [build_repo])
            if build_repo_only:
                build_list = build_repo
            common.call("%srosmake -rV --profile %s--test-only --output=%s %s" %
                        ("/opt/VirtualGL/bin/vglrun " if graphic_test else "", 
                         "--pjobs=8 " if not graphic_test else "",
                         dry_build_logs, build_list ), ros_env_repo)
        except common.BuildException as ex:
            print ex.msg

        # copy test results
        common.call("rosrun rosunit clean_junit_xml.py", ros_env_repo)
        for file in os.listdir(os.path.join(repo_sourcespace, "test_results")):
            file_path = os.path.join(repo_sourcespace, "test_results", file)
            print file_path
            try:
                if not file.startswith("_hudson"):
                    shutil.rmtree(file_path)
            except Exception as e:
                print e

        common.copy_test_results(workspace, repo_sourcespace)
        print datetime.datetime.now()
        try:
            shutil.move(dry_build_logs, os.path.join(workspace, "build_logs"))
        except IOError as ex:
            print "No build logs found: %s" % ex


if __name__ == "__main__":
    # global try
    try:
        main()
        print "Test script finished cleanly!"

    # global catch
    except (common.BuildException, cob_pipe.CobPipeException) as ex:
        print "Test script failed!"
        print ex.msg
        raise ex

    except Exception as ex:
        print "Test script failed! Check out the console output above for details."
        raise ex
