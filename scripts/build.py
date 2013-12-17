#!/usr/bin/env python

import optparse
import sys
import os
import shutil
import datetime
import traceback

from jenkins_setup import common, rosdep, cob_pipe, cppcheck

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
    print "\nTesting on ros distro:  %s" % ros_distro
    print "Testing repository: %s" % build_repo
    if build_repo != build_identifier:
        print "       with suffix: %s" % build_identifier.split('__')[1]
    print "Using source: %s" % pipe_repos[build_identifier].url
    print "Testing branch/version: %s" % pipe_repos[build_identifier].version
    print "\n", 50 * 'X'

    # set up directories variables
    tmpdir = os.path.join('/tmp', 'test_repositories')
    repo_sourcespace = os.path.join(tmpdir, 'src_repository')                      # location to store repositories in
    repo_sourcespace_wet = os.path.join(tmpdir, 'src_repository', 'wet', 'src')    # wet (catkin) repositories
    os.makedirs(repo_sourcespace_wet)
    repo_sourcespace_dry = os.path.join(tmpdir, 'src_repository', 'dry')           # dry (rosbuild) repositories
    os.makedirs(repo_sourcespace_dry)
    repo_static_analysis_results = os.path.join(tmpdir, 'src_repository', 'static_analysis_results') # location for static code test results
    os.makedirs(repo_static_analysis_results)
    dry_build_logs = os.path.join(repo_sourcespace_dry, 'build_logs')              # location for build logs
    os.makedirs(dry_build_logs)

    # setup ros workspace and setup environment variables
    print "Set up ros workspace and setup environment variables"
    ros_env_repo = common.get_ros_env('/opt/ros/%s/setup.bash' % ros_distro)
    common.call("catkin_init_workspace %s" % repo_sourcespace_wet, ros_env_repo)
    os.chdir(repo_sourcespace_wet + "/..")
    common.call("catkin_make", envir = ros_env_repo, verbose = False) # this means running "make" on an empty catkin workspace to generate a setup.bash
    ros_env_repo = common.get_ros_env(os.path.join(repo_sourcespace_wet, '../devel/setup.bash')) # source catkin workspace
    ros_env_repo['ROS_PACKAGE_PATH'] = ':'.join([repo_sourcespace_dry, os.environ['ROS_PACKAGE_PATH']]) # add dry path to ROS_PACKAGE_PATH

    ################
    ### checkout ###
    ################
    time_checkout = datetime.datetime.now()
    print "=====> entering checkout step at", time_checkout

    # download build_repo from source #
    print "Creating rosinstall file for repository %s" % build_repo
    rosinstall = ""
    if build_identifier in pipe_repos:  # check if triggering identifier is really present in pipeline config
        rosinstall += pipe_repos[build_identifier].get_rosinstall()
    else:
        err_msg = "Pipeline was triggered by repository %s which is not in pipeline config!" % build_identifier
        raise common.BuildException(err_msg)

    # write rosinstall file
    print "Rosinstall file for repository: \n %s" % rosinstall
    with open(os.path.join(workspace, 'repo.rosinstall'), 'w') as f:
        f.write(rosinstall)
    print "Install repository from source:"
    # rosinstall repos
    common.call("rosinstall -j 8 --verbose --continue-on-error %s %s/repo.rosinstall /opt/ros/%s"
                % (repo_sourcespace, workspace, ros_distro))

    # get the repositories build dependencies
    print "Get build dependencies of repo"

    # get all packages in sourcespace
    (catkin_packages, stacks, manifest_packages) = common.get_all_packages(repo_sourcespace)

    # (debug) output
    if options.verbose:
        print "Packages in %s:" % repo_sourcespace
        print "Catkin: ", catkin_packages
        print "Rosbuild:\n  Stacks: ", stacks
        print "  Packages: ", manifest_packages

        # get deps directly for catkin (like in willow code)
        try:
            print "Found wet build dependencies:\n%s" % '- ' + '\n- '.join(sorted(common.get_dependencies(repo_sourcespace, build_depends=True, test_depends=False)))
        except:
            pass
        # deps catkin
        repo_build_dependencies = common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
        print "Found wet dependencies:\n%s" % '- ' + '\n- '.join(sorted(repo_build_dependencies))
        # deps stacks
        repo_build_dependencies = common.get_nonlocal_dependencies({}, stacks, {})
        print "Found dry dependencies:\n%s" % '- ' + '\n- '.join(sorted(repo_build_dependencies))

    # check if build_repo is wet or dry and get all corresponding deps
    build_repo_type = ''
    if build_repo in catkin_packages:
        print "repo %s is wet" % build_repo
        build_repo_type = 'wet'
        repo_build_dependencies = common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
    elif build_repo in stacks:
        print "repo %s is dry" % build_repo
        build_repo_type = 'dry'
        repo_build_dependencies = common.get_nonlocal_dependencies({}, stacks, {})
    else:
        # build_repo is neither wet nor dry
        raise common.BuildException("Repository %s to build not found in sourcespace" % build_repo)

    # install user-defined/customized dependencies of build_repo from source
    rosinstall = ''
    fulfilled_deps = []
    for dep in repo_build_dependencies:
        if dep in pipe_repos[build_identifier].dependencies:
            print "Install user-defined build dependency %s from source" % dep
            rosinstall += pipe_repos[build_identifier].dependencies[dep].get_rosinstall()
            fulfilled_deps.append(dep)

    # install additional, indirect user-defined dependencies
    for dep in pipe_repos[build_identifier].dependencies:
        if dep not in fulfilled_deps:
            print "Install additional user-defined build dependency %s from source" % dep
            rosinstall += pipe_repos[build_identifier].dependencies[dep].get_rosinstall()
            fulfilled_deps.append(dep)

    # check if all user-defined/customized dependencies are satisfied
    if sorted(fulfilled_deps) != sorted(pipe_repos[build_identifier].dependencies):
        print "Not all user-defined build dependencies are fulfilled"
        print "User-defined build dependencies:\n - %s" % '\n - '.join(pipe_repos[build_identifier].dependencies)
        print "Fulfilled dependencies:\n - %s" % '\n - '.join(fulfilled_deps)
        raise common.BuildException("Not all user-defined build dependencies are fulfilled")

    if rosinstall != '':
        # write .rosinstall file
        print "Rosinstall file for user-defined build dependencies: \n %s" % rosinstall
        with open(os.path.join(workspace, "repo.rosinstall"), 'w') as f:
            f.write(rosinstall)
        print "Install user-defined build dependencies from source"
        # rosinstall depends
        common.call("rosinstall -j 8 --verbose --continue-on-error %s %s/repo.rosinstall /opt/ros/%s"
                    % (repo_sourcespace, workspace, ros_distro))

        # get also deps of just installed user-defined/customized dependencies
        (catkin_packages, stacks, manifest_packages) = common.get_all_packages(repo_sourcespace)
        if build_repo_type == 'wet':
            if stacks != {}:
                raise common.BuildException("Catkin (wet) package %s depends on (dry) stack(s):\n%s"
                                            % (build_repo, '- ' + '\n- '.join(stacks)))
            # take only wet packages
            repo_build_dependencies = common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
        else:  # dry build repo
            # take all packages
            repo_build_dependencies = common.get_nonlocal_dependencies(catkin_packages, stacks, {}, build_depends=True, test_depends=False)
        repo_build_dependencies = [dep for dep in repo_build_dependencies if dep not in fulfilled_deps]

    # separate installed repos in wet and dry
    print "Separate installed repositories in wet and dry"
    # get all folders in repo_sourcespace
    sourcespace_dirs = [name for name in os.listdir(repo_sourcespace) if os.path.isdir(os.path.join(repo_sourcespace, name))]
    for dir in sourcespace_dirs:
        if dir in catkin_packages.keys():
            shutil.move(os.path.join(repo_sourcespace, dir), os.path.join(repo_sourcespace_wet, dir))
        if dir in stacks.keys():
            shutil.move(os.path.join(repo_sourcespace, dir), os.path.join(repo_sourcespace_dry, dir))

    ############################
    ### install dependencies ###
    ############################
    time_install = datetime.datetime.now()
    print "=====> entering dependency installation step at", time_install

    # Create rosdep object
    rosdep_resolver = None
    print "Create rosdep object"
    try:
        rosdep_resolver = rosdep.RosDepResolver(ros_distro)
    except:  # when init fails the first time
        from time import sleep
        sleep(10)
        rosdep_resolver = rosdep.RosDepResolver(ros_distro)

    print "Install build dependencies: %s" % (', '.join(repo_build_dependencies))
    common.apt_get_install_also_nonrosdep(repo_build_dependencies, ros_distro, rosdep_resolver)

    #print "Install wet rosdep dependencies" # TODO: this throws an error if the dependency is not a system dependency or a released package. How can we handle wet dependencies which are in the same local workspace?
    #for dir in catkin_packages.keys():
    #    common.call("rosdep install -y %s" % dir, ros_env_repo)

    print "Install dry rosdep dependencies"
    for dir in stacks.keys():
        common.call("rosdep install -y %s" % dir, ros_env_repo)

    #######################
    ### static analysis ###
    #######################
    time_analysis = datetime.datetime.now()
    print "=====> entering static analysis step at", time_analysis

    #TODO
    # Cpplint
    # Counting lines of code/comments
    # cppcheck
    cppcheck.run([repo_sourcespace_wet, repo_sourcespace_dry], repo_static_analysis_results)
    # Coverage

    # create tests results directory in workspace
    os.mkdir(workspace + "/static_analysis_results")
    
    # copy test results
    common.copy_static_analysis_results(repo_static_analysis_results, workspace + "/static_analysis_results")

    #############
    ### build ###
    #############
    time_build = datetime.datetime.now()
    print "=====> entering build step at", time_build

    ### catkin repositories
    if catkin_packages != {}:
        os.chdir(repo_sourcespace_wet + "/..")
        try:
            common.call("catkin_make", ros_env_repo)
        except common.BuildException as ex:
            print ex.msg
            raise common.BuildException("Failed to catkin_make wet repositories")

    ### rosbuild repositories
    if build_repo_type == 'dry':
        # build dry repositories
        print "Build repository %s" % build_repo
        try:
            common.call("rosmake -rV --skip-blacklist --profile --pjobs=8 --output=%s %s" %
                        (dry_build_logs, build_repo), ros_env_repo)
        except common.BuildException as ex:
            try:
                shutil.move(dry_build_logs, os.path.join(workspace, "build_logs"))
            finally:
                print ex.msg
                raise common.BuildException("Failed to rosmake %s" % build_repo)

    # the end (steps: parsing, checkout, install, analysis, build, finish)
    time_finish = datetime.datetime.now()
    print "=====> finished script at", time_finish
    print "durations:"
    print "parsing arguments in       ", (time_checkout - time_parsing)
    print "checkout in                ", (time_install - time_checkout)
    print "install dependencies in    ", (time_analysis - time_install)
    print "static code analysis in    ", (time_build - time_analysis)
    print "build in                   ", (time_finish - time_build)
    print "total                      ", (time_finish - time_parsing)
    print ""

if __name__ == "__main__":
    # global try
    try:
        main()
        print "Build script finished cleanly!"

    # global catch
    except (common.BuildException, cob_pipe.CobPipeException) as ex:
        print traceback.format_exc()
        print "Build script failed!"
        print ex.msg
        raise ex

    except Exception as ex:
        print traceback.format_exc()
        print "Build script failed! Check out the console output above for details."
        raise ex
