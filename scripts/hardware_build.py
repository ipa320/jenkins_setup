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
    cp_instance.load_config_from_file(pipeline_repos_owner, server_name, user_name, file_location=os.environ["WORKSPACE"])
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

    # for hardware build: only support hydro
    if ros_distro == "groovy":
        sys.exit(False)

    # for hardware build: remove old build artifacts 
    limit = 3 # limit amount of old builds
    print workspace
    while len(os.listdir(workspace + "/..")) > limit + 1: # we will not count the "current" sym link
        list = os.listdir(workspace + "/..")
        shutil.rmtree(workspace + "/../" + sorted(list)[0])

    # set up directories variables
    tmpdir = workspace
    repo_checkoutspace = os.path.join(tmpdir, 'checkout')                          # location to store repositories in (will be separated in wet and dry later on)
    os.makedirs(repo_checkoutspace)
    repo_sourcespace = os.path.join(tmpdir, 'src')                                 # location to build repositories in
    os.makedirs(repo_sourcespace)
    repo_sourcespace_wet = os.path.join(repo_sourcespace, 'wet', 'src')            # wet (catkin) repositories
    os.makedirs(repo_sourcespace_wet)
    repo_sourcespace_dry = os.path.join(repo_sourcespace, 'dry')                   # dry (rosbuild) repositories
    os.makedirs(repo_sourcespace_dry)
    repo_build_logs = os.path.join(tmpdir, 'build_logs')                           # location for build logs
    os.makedirs(repo_build_logs)

    # init catkin workspace
    ros_env_repo = common.get_ros_env('/opt/ros/%s/setup.bash' % ros_distro)                            # source ros_distro (needed to do a catkin_init_workspace)
    # for hardware build: merge workspace with robot account #FIXME: this should be parameterisable in plugin (select a path to a setup.bash file in the admin config and mark a checkbox for the user config)
    if os.path.isdir("/u/robot/git/care-o-bot/devel"):
        ros_env_repo = common.get_ros_env('/u/robot/git/care-o-bot/devel/setup.bash')
    common.call("catkin_init_workspace %s" % repo_sourcespace_wet, ros_env_repo, verbose=False)         # init wet workspace
    common.call("catkin_make --directory %s/wet" % repo_sourcespace, ros_env_repo, verbose=False)       # build wet workspace to generate a setup.bash

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
    common.call("rosinstall -j 8 --verbose %s %s/repo.rosinstall /opt/ros/%s"
                % (repo_checkoutspace, workspace, ros_distro))

    # get the repositories build dependencies
    print "Get build dependencies of repo"

    # get all packages in checkoutspace
    (catkin_packages, stacks, manifest_packages) = common.get_all_packages(repo_checkoutspace)

    # check if build_repo is wet or dry and get all corresponding deps
    build_repo_type = ''
    if build_repo in catkin_packages: # wet repo with metapackage
        print "repo %s is wet" % build_repo
        build_repo_type = 'wet'
        repo_build_dependencies = common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
    elif (build_repo not in catkin_packages) and (build_repo not in stacks) and (catkin_packages != []): # wet repo without metapackage
        print "repo %s is wet without metapackage" % build_repo
        build_repo_type = 'wet'
        repo_build_dependencies = common.get_nonlocal_dependencies(catkin_packages, {}, {}, build_depends=True, test_depends=False)
    elif build_repo in stacks: # dry repo with stack
        print "repo %s is dry" % build_repo
        build_repo_type = 'dry'
        repo_build_dependencies = common.get_nonlocal_dependencies({}, stacks, {})
    #TODO elif : # dry repo without stack
    else: # build_repo is neither wet nor dry
        raise common.BuildException("Repository %s to build not found in checkoutspace" % build_repo)

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
        common.call("rosinstall -j 8 --verbose %s %s/repo.rosinstall /opt/ros/%s"
                    % (repo_checkoutspace, workspace, ros_distro))

        # get also deps of just installed user-defined/customized dependencies
        (catkin_packages, stacks, manifest_packages) = common.get_all_packages(repo_checkoutspace)

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

    print ""
    print "Found the following packages"
    print "  wet packages:     ", catkin_packages.keys()
    print "  dry stacks:       ", stacks.keys()
    print "  dry packages:     ", manifest_packages.keys()
    print ""

    # separate installed repos in wet and dry
    print "Separate installed repositories in wet and dry"
    # get all folders in repo_checkoutspace
    checkoutspace_dirs = [name for name in os.listdir(repo_checkoutspace) if os.path.isdir(os.path.join(repo_checkoutspace, name))]
    for dir in checkoutspace_dirs:
        if dir in catkin_packages.keys(): # wet repo with metapackage
            shutil.move(os.path.join(repo_checkoutspace, dir), os.path.join(repo_sourcespace_wet, dir))
        elif build_repo_type == 'wet' and dir == build_repo: # wet repo without metapackage
            shutil.move(os.path.join(repo_checkoutspace, dir), os.path.join(repo_sourcespace_wet, dir))
        elif dir in stacks.keys(): # dry repo with stack
            shutil.move(os.path.join(repo_checkoutspace, dir), os.path.join(repo_sourcespace_dry, dir))
        #TODO elif: # dry repo without stack
        #else:
        #    raise common.BuildException("Could not separate %s into wet or dry sourcespace." %dir) 
    # remove checkout dir
    common.call("rm -rf %s" % repo_checkoutspace)

    # setup ros workspace
    print "Set up ros workspace and setup environment variables"
    ros_env_repo = common.get_ros_env('/opt/ros/%s/setup.bash' % ros_distro)
    common.call("rosws init %s /opt/ros/%s" %(repo_sourcespace, ros_distro), verbose=False)             # init workspace pointing to ros_distro

    # for hardware build: merge workspace with robot account #FIXME: this should be parameterisable in plugin (select a path to a setup.bash file in the admin config and mark a checkbox for the user config)
    if os.path.isdir("/u/robot/git/care-o-bot"):
        common.call("rosws merge -t %s /u/robot/git/care-o-bot" %repo_sourcespace, verbose=False)      # merge robot account workspace

    common.call("rosws merge -t %s %s/wet/devel" % (repo_sourcespace, repo_sourcespace), verbose=False) # merge wet workspace
    common.call("rosws merge -t %s %s/dry" % (repo_sourcespace, repo_sourcespace), verbose=False)       # merge dry workspace
    ros_env_repo = common.get_ros_env(repo_sourcespace + '/setup.bash')                                 # source wet and dry workspace

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
    missing_packages = common.apt_get_check_also_nonrosdep(repo_build_dependencies, ros_distro, rosdep_resolver)
    if len(missing_packages) > 0:
        raise common.BuildException("Some dependencies are missing. Please ask your administrator to install the following packages: %s" % missing_packages)
    print "All denendencies already installed."

    #############
    ### build ###
    #############
    time_build = datetime.datetime.now()
    print "=====> entering build step at", time_build

    # get amount of cores available on host system
    cores = multiprocessing.cpu_count()

    ### catkin repositories
    if catkin_packages != {}:
        print "Build wet packages: ", catkin_packages.keys()
        try:
            common.call("catkin_make --directory %s/wet" % repo_sourcespace, ros_env_repo)
        except common.BuildException as ex:
            print ex.msg
            raise common.BuildException("Failed to catkin_make wet repositories")

    ### rosbuild repositories
    if build_repo_type == 'dry':
        # build dry repositories
        print "Build dry stacks:   ", stacks.keys()
        print "Build dry packages: ", manifest_packages.keys()
        packages_to_build = " ".join(stacks.keys()) + " ".join(manifest_packages.keys())
        try:
            common.call("rosmake -rV --skip-blacklist --profile --pjobs=%s --output=%s %s" %
                        (cores + 1, repo_build_logs, packages_to_build), ros_env_repo)
        except common.BuildException as ex:
            try:
                shutil.move(repo_build_logs, os.path.join(workspace, "build_logs"))
            finally:
                print ex.msg
                raise common.BuildException("Failed to rosmake dry repositories")

    ###########
    ### end ###
    ###########
    # for hardware builds: create sym link to new build
    common.call("rm -f %s/../current" % workspace)
    common.call("ln -sf %s %s/../current" %(workspace, workspace))
    
    # steps: parsing, checkout, install, analysis, build, finish
    time_finish = datetime.datetime.now()
    print "=====> finished script at", time_finish
    print "durations:"
    print "parsing arguments in       ", (time_checkout - time_parsing)
    print "checkout in                ", (time_install - time_checkout)
    print "install dependencies in    ", (time_build - time_install)
    print "build in                   ", (time_finish - time_build)
    print "total                      ", (time_finish - time_parsing)
    print ""
    print "For manual testing, please run the following line in your testing terminal"
    print "source " + repo_sourcespace + "/setup.bash"
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
