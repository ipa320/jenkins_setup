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
    cp_instance.load_config_from_file(pipeline_repos_owner, server_name, user_name, file_location=os.environ["WORKSPACE"])
    pipe_repos = cp_instance.repositories
    common.output("Pipeline configuration successfully loaded", blankline='b')

    # (debug) output
    print "\n", 50 * 'X'
    print "\nTesting on ros distro:  %s" % ros_distro
    print "Testing repository: %s" % build_repo
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

    # get environment variables    
    ros_env_repo = common.get_ros_env(os.path.join(repo_sourcespace, 'wet', 'devel', 'setup.bash'))
    ros_env_repo['ROS_PACKAGE_PATH'] = ':'.join([repo_sourcespace, ros_package_path])

    ############
    ### test ###
    ############
    time_test = datetime.datetime.now()
    print "=====> entering testing step at", time_test
    
    # FIXME: HACK BEGIN
    
    common.call("ls", envir=ros_env_repo, verbose=False)
    common.call("roslaunch fiad_scenario_system system.launch", envir=ros_env_repo, verbose=False)
    
    # FIXME: HACK END
    
    # the end (steps: parsing, test)
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
        traceback.format_exc()
        print "Test script failed!"
        print ex.msg
        raise ex

    except Exception as ex:
        traceback.format_exc()
        print "Test script failed! Check out the console output above for details."
        raise ex
