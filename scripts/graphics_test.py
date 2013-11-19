#!/usr/bin/env python

import optparse
import sys
import os
import shutil
import datetime
import traceback

from jenkins_setup import common, rosdep, cob_pipe


def main():
    #########################
    ### parsing arguments ###
    #########################
    time_parsing = datetime.datetime.now()
    print "=====> entering argument parsing step at", time_parsing

    # parse parameter values
    parser = optparse.OptionParser()
    parser.add_option('-v', '--verbose', action='store_true', default=False)
    (options, args) = parser.parse_args()

    if len(args) < 5:
        print "Usage: %s pipeline_repos_owner server_name user_name ros_distro build_repo" % sys.argv[0]
        raise common.BuildException("Wrong arguments for build script")

    # get arguments
    pipeline_repos_owner = args[0]
    server_name = args[1]
    user_name = args[2]
    ros_distro = args[3]
    build_identifier = args[4]                      # repository + suffix
    build_repo = build_identifier.split('__')[0]    # only repository to build
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
    print "Graphic Test: True"
    if build_repo != build_identifier:
        print "       with suffix: %s" % build_identifier.split('__')[1]
    print "Using source: %s" % pipe_repos[build_identifier].url
    print "Testing branch/version: %s" % pipe_repos[build_identifier].version
    print "\n", 50 * 'X'

    # set up directories variables
    tmpdir = os.path.join('/tmp', 'test_repositories')
    repo_sourcespace = os.path.join(tmpdir, 'src_repository')                                         # location to store repositories in
    repo_sourcespace_wet = os.path.join(tmpdir, 'src_repository', 'wet', 'src')                       # wet (catkin) repositories
    repo_sourcespace_dry = os.path.join(tmpdir, 'src_repository', 'dry')                              # dry (rosbuild) repositories
    repo_test_results_dry = os.path.join(tmpdir, 'src_repository', 'test_results')                    # location for dry test results
    repo_test_results_wet = os.path.join(tmpdir, 'src_repository', 'wet', 'build', 'test_results')    # location for wet test results
    #repo_buildspace = os.path.join(tmpdir, 'build_repository')                                        # location for build output
    dry_build_logs = os.path.join(repo_sourcespace_dry, 'build_logs')                                 # location for build logs

    if ros_distro != 'electric':
        # Create rosdep object
        print "Create rosdep object"
        try:
            rosdep_resolver = rosdep.RosDepResolver(ros_distro)
        except:  # when init fails the first time
            from time import sleep
            sleep(10)
            rosdep_resolver = rosdep.RosDepResolver(ros_distro)

    ## env
    #print "Set up ros environment variables"
    #ros_env = common.get_ros_env('/opt/ros/%s/setup.bash' % ros_distro)
    #if options.verbose:
    #    common.call("env", ros_env)
    
    ros_env_repo = common.get_ros_env(os.path.join(repo_sourcespace, 'setup.bash'))
    ros_env_repo['ROS_PACKAGE_PATH'] = ':'.join([repo_sourcespace, ros_package_path])
    if options.verbose:
        common.call("env", ros_env_repo)

    ############
    ### test ###
    ############
    time_test = datetime.datetime.now()
    print "=====> entering testing step at", time_test

    ### catkin repositories
    print "test catkin repositories"
    ros_env_repo['ROS_TEST_RESULTS_DIR'] = repo_test_results_wet
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
            #common.call("catkin_init_workspace %s" % repo_sourcespace_wet, ros_env_repo)

        # test repositories
        #print "Test wet repository list"

        #if not os.path.isdir(repo_buildspace):
        #    os.mkdir(repo_buildspace)
        os.chdir(repo_sourcespace_wet + "/..")

        #test_error_msg = None
        try:
            common.call("/opt/VirtualGL/bin/vglrun catkin_make test", ros_env_repo)
        except common.BuildException as ex:
            print ex.msg
            #print traceback.format_exc()
        #    test_error_msg = ex.msg

        # get wet repositories test and run dependencies
        #print "Get test and run dependencies of repo list"
        #(catkin_packages, stacks, manifest_packages) = common.get_all_packages(repo_sourcespace_wet)
        #if stacks != {}:
        #    raise common.BuildException("Catkin (wet) package %s depends on (dry) stack(s):\n%s"
        #                                % (build_repo, '- ' + '\n- '.join(stacks)))
        # take only wet packages
        #repo_test_dependencies = common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=False, test_depends=True)
        #if repo_test_dependencies != [] and test_error_msg is None:
        #    print "Install test and run dependencies of repository list: %s" % (', '.join(repo_test_dependencies))
        #    common.apt_get_install_also_nonrosdep(repo_test_dependencies, ros_distro, rosdep_resolver)

        #    # run tests
        #    print "Test repository list"
        #    try:
        #        common.call("%smake run_tests" % ("/opt/VirtualGL/bin/vglrun " if graphic_test else ""), ros_env_repo)  # TODO check how to test a list of repos
        #    except common.BuildException as ex:
        #        print ex.msg
        #        test_error_msg = ex.msg

        # clean test xml files
        common.call("rosrun rosunit clean_junit_xml.py", ros_env_repo)
        for file in os.listdir(os.path.join(repo_test_results_wet)):
            file_path = os.path.join(repo_test_results_wet, file)
            try:
                if not file.startswith("_hudson"):
                    shutil.rmtree(file_path)
            except Exception as e:
                print e

        # copy wet test results
        common.copy_test_results(repo_test_results_wet, workspace + "/test_results")

    ### rosbuild repositories
    print "test rosbuild repositories"
    if not os.path.isdir(repo_test_results_dry):
        os.mkdir(repo_test_results_dry)
    ros_env_repo['ROS_TEST_RESULTS_DIR'] = repo_test_results_dry
    (catkin_packages, stacks, manifest_packages) = common.get_all_packages(repo_sourcespace_dry)
    if build_repo in stacks:
        # get list of dependencies to test
        test_repos_list = []
        for dep, depObj in pipe_repos[build_identifier].dependencies.items():
            if depObj.test and dep in stacks:  # TODO option to select deps to build
                test_repos_list.append(dep)

        # test dry repositories
        #print "Test repository %s" % build_repo
        try:
            build_list = " ".join(test_repos_list + [build_repo])
            common.call("/opt/VirtualGL/bin/vglrun rosmake -rV --profile --test-only --output=%s %s" %
                        ( dry_build_logs, build_list ), ros_env_repo)
        except common.BuildException as ex:
            print ex.msg

        # clean test xml files
        common.call("rosrun rosunit clean_junit_xml.py", ros_env_repo)
        for file in os.listdir(os.path.join(repo_test_results_dry)):
            file_path = os.path.join(repo_test_results_dry, file)
            try:
                if not file.startswith("_hudson"):
                    shutil.rmtree(file_path)
            except Exception as e:
                print e

        # copy dry test results
        common.copy_test_results(repo_test_results_dry, workspace + "/test_results")
    
    #print datetime.datetime.now()
    #try:
    #    shutil.move(dry_build_logs, os.path.join(workspace, "build_logs"))
    #except IOError as ex:
    #    print "No build logs found: %s" % ex

    # the end (steps: parsing, test)
    time_finish = datetime.datetime.now()
    print "=====> finished script at", time_finish
    print "durations:"
    print "parsing arguments in       ", (time_test - time_parsing)
    print "test in                    ", (time_finish - time_test)
    print ""


if __name__ == "__main__":
    # global try
    try:
        main()
        print "Test script finished cleanly!"

    # global catch
    except (common.BuildException, cob_pipe.CobPipeException) as ex:
        traceback.format_exc()
        print "Test script failed!"
        print ex.msg
        raise ex

    except Exception as ex:
        traceback.format_exc()
        print "Test script failed! Check out the console output above for details."
        raise ex
