#!/usr/bin/env python

import optparse
import sys
import os
import shutil
import datetime
import traceback
import multiprocessing

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

    # cob_pipe object
    cp_instance = cob_pipe.CobPipe()
    cp_instance.load_config_from_url(pipeline_repos_owner, server_name, user_name)
    pipe_repos = cp_instance.repositories
    common.output("Pipeline configuration successfully loaded", blankline='b')

    # (debug) output
    print "\n", 50 * 'X'
    print "\nTesting on ros distro: %s" % ros_distro
    print "Testing repository: %s" % build_repo
    if build_repo != build_identifier:
        print "       with suffix: %s" % build_identifier.split('__')[1]
    print "Using source: %s" % pipe_repos[build_identifier].url
    print "Testing branch/version: %s" % pipe_repos[build_identifier].version
    print "\n", 50 * 'X'

    # set up directories variables
    tmpdir = '/tmp'
    repo_sourcespace = os.path.join(tmpdir, 'src')                                 # location to build repositories in
    repo_sourcespace_wet = os.path.join(repo_sourcespace, 'wet', 'src')            # wet (catkin) repositories
    repo_sourcespace_dry = os.path.join(repo_sourcespace, 'dry')                   # dry (rosbuild) repositories
    repo_test_results = os.path.join(tmpdir, 'test_results')                       # location for test results
    if not os.path.exists(repo_test_results):
        os.makedirs(repo_test_results)
    repo_build_logs = os.path.join(tmpdir, 'build_logs')                           # location for build logs

    # source wet and dry workspace
    ros_env_repo = common.get_ros_env(repo_sourcespace + '/setup.bash')
    ros_env_repo['ROS_TEST_RESULTS_DIR'] = repo_test_results

    ############
    ### test ###
    ############
    time_test = datetime.datetime.now()
    print "=====> entering testing step at", time_test

    # get amount of cores available on host system
    cores = multiprocessing.cpu_count()

    ### catkin repositories
    print "test catkin repositories"
    if os.listdir(repo_sourcespace_wet):
        # get wet repositories test and run dependencies
        #print "Get test and run dependencies of repo list"
        (catkin_packages, stacks, manifest_packages) = common.get_all_packages(repo_sourcespace_wet)
        # get list of dependencies to test
        test_repos_list_wet = []
        for dep, depObj in pipe_repos[build_identifier].dependencies.items():
            if depObj.test and dep in catkin_packages:
                test_repos_list_wet.append(dep)

        print "Test the following wet repositories %s" % test_repos_list_wet

        #test_error_msg = None
        try:
            test_list = ' '.join( test_repos_list_wet )
            if test_list:
                common.call( "/opt/VirtualGL/bin/vglrun catkin_make test --pkg %s" % test_list, ros_env_repo)

        except common.BuildException as ex:
            print ex.msg
            #print traceback.format_exc()
        #    test_error_msg = ex.msg

        # clean test xml files
        common.call("rosrun rosunit clean_junit_xml.py", ros_env_repo)
        for file in os.listdir(os.path.join(repo_test_results)):
            file_path = os.path.join(repo_test_results, file)
            try:
                if not file.startswith("_hudson"):
                    shutil.rmtree(file_path)
            except Exception as e:
                print e

        # copy wet test results
        common.copy_test_results(repo_test_results, workspace + "/test_results")

    ### rosbuild repositories
    print "test rosbuild repositories"
    (catkin_packages, stacks, manifest_packages) = common.get_all_packages(repo_sourcespace_dry)
    if build_repo in stacks:
        # get list of dependencies to test
        test_repos_list_dry = [build_repo]
        for dep, depObj in pipe_repos[build_identifier].dependencies.items():
            if depObj.test and dep in stacks:
                test_repos_list_dry.append(dep)

        # test dry repositories
        print "Test the following dry repositories %s" % test_repos_list_dry
        try:
            build_list = " ".join(test_repos_list_dry)
            common.call("/opt/VirtualGL/bin/vglrun rosmake -rV --skip-blacklist --profile --pjobs=%s --test-only --output=%s %s" %
                        ( cores, repo_build_logs, build_list ), ros_env_repo)
        except common.BuildException as ex:
            print ex.msg

        # clean test xml files
        common.call("rosrun rosunit clean_junit_xml.py", ros_env_repo)
        for file in os.listdir(os.path.join(repo_test_results)):
            file_path = os.path.join(repo_test_results, file)
            try:
                if not file.startswith("_hudson"):
                    shutil.rmtree(file_path)
            except Exception as e:
                print e

        # copy dry test results
        common.copy_test_results(repo_test_results, workspace + "/test_results")

    ###########
    ### end ###
    ###########
    # steps: parsing, test
    time_finish = datetime.datetime.now()
    print "=====> finished script at", time_finish
    print "durations:"
    print "parsing arguments in       ", (time_test - time_parsing)
    print "test in                    ", (time_finish - time_test)
    print "total                      ", (time_finish - time_parsing)
    print ""

if __name__ == "__main__":
    # global try
    try:
        main()
        print "Test script finished cleanly!"

    # global catch
    except (common.BuildException, cob_pipe.CobPipeException) as ex:
        print traceback.format_exc()
        print "Test script failed!"
        print ex.msg
        raise ex

    except Exception as ex:
        print traceback.format_exc()
        print "Test script failed! Check out the console output above for details."
        raise ex
