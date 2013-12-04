#!/usr/bin/env python2.7

import paramiko
import urllib2
import contextlib
import os
import subprocess
import sys
import fnmatch
import yaml
#import threading
import time
#from Queue import Queue
#from threading import Thread


def append_pymodules_if_needed():
    """
    Add pymodules to path if not already included.
    """
    #TODO: This is a hack, in the chroot, the default python path does not
    if not os.path.abspath("/usr/lib/pymodules/python2.7") in sys.path:
        sys.path.append("/usr/lib/pymodules/python2.7")


def apt_get_update(sudo=False):
    """
    Update apt-get.

    @param sudo: call command with sudo (default False)
    @type  sudo: bool
    """
    if not sudo:
        call("apt-get update")
    else:
        call("sudo apt-get update")


def apt_get_install(pkgs, rosdep=None, sudo=False):
    """
    Install the corresponding apt packages from a list of ROS repositories.

    @param pkgs: names of ros repositories
    @type  pkgs: list
    @param rosdep: rosdep resolver object (default None)
    @type  rosdep: rosdep.RosDepResolver
    @param sudo: execute command as super-user (default False)
    @type  sudo: bool
    """
    cmd = "apt-get install --yes --force-yes "
    if sudo:
        cmd = "sudo " + cmd

    if len(pkgs) > 0:
        if rosdep:
            call(cmd + ' '.join(rosdep.to_aptlist(pkgs)))
        else:
            call(cmd + ' '.join(pkgs))
    else:
        print "Not installing anything from apt right now."


def apt_get_install_also_nonrosdep(pkgs, ros_distro, rosdep=None, sudo=False):
    """
    Extend common.apt_get_install by trying to guess Debian package names
    of packages not included in rosdep

    @param pkgs: names of ros repositories
    @type  pkgs: list
    @param ros_distro: name of ros release, e.g. fuerte
    @type  ros_distro: str
    @param rosdep: rosdep resolver object (default None)
    @type  rosdep: rosdep.RosDepResolver
    @param sudo: execute command as super-user (default False)
    @type  sudo: bool
    """
    rosdep_pkgs = []
    aptget_pkgs = []
    unavailable_pkgs = []

    import apt
    for pkg in pkgs:
        if rosdep and rosdep.has_ros(pkg):
            debian_pkgs = rosdep.to_apt(pkg)
            rosdep_pkgs.append(pkg)
        else:
            debian_pkgs = ['-'.join(['ros', ros_distro, pkg.replace('_', '-')])]
            aptget_pkgs += debian_pkgs
        # use python apt module to check if Debian package exists
        for debian_pkg in debian_pkgs:
            if debian_pkg not in apt.Cache():
                unavailable_pkgs.append(debian_pkg)

    print ""
    print "apt dependencies: ", aptget_pkgs
    print "ros dependencies: ", rosdep_pkgs
    print "unavailable dependencies: ", unavailable_pkgs
    print ""

    if unavailable_pkgs != []:
        raise BuildException("Some dependencies are not available: %s" % (', '.join(unavailable_pkgs)))

    if aptget_pkgs != []:
        try:
            print "Installing apt dependencies"
            apt_get_install(aptget_pkgs, sudo=sudo)
        except:
            raise BuildException("Failed to apt-get install apt dependencies")

    if rosdep_pkgs != []:
        try:
            print "Installing ros dependencies"
            apt_get_install(rosdep_pkgs, rosdep, sudo)
        except:
            raise BuildException("Failed to apt-get install ros dependencies")


def copy_test_results(buildspace_test_results_dir, workspace_test_results_dir, errors=None, prefix='dummy'):
    """
    Copy test results from buildspace into workspace or create dummy.xml.

    @param workspace_test_results_dir: path the test results will copied into
    @type  workspace_test_results_dir: str
    @param buildspace_test_results_dir: path where the test results are stored
    @type  buildspace_test_results_dir: str
    @param errors: error name in the dummy file (default None)
    @type  errors: str
    @param prefix: prefix in the dummy file (default dummy)
    @type  prefix: str
    """
    #print "Preparing xml test results"
    try:
        os.makedirs(os.path.join(workspace_test_results_dir))
        print "Created test results directory"
    except:
        pass
    os.chdir(workspace_test_results_dir)
    print "Copy all test results to " + workspace_test_results_dir

    # copy all rostest test reports nested in their packagename's directory    
    count = 0
    for root, dirnames, filenames in os.walk(buildspace_test_results_dir):
        for filename in fnmatch.filter(filenames, '*.xml'):
            call("cp %s %s" % (os.path.join(root, filename), workspace_test_results_dir))
            count += 1

	# create dummy test if no rostest result exists
    if count == 0:
        print "No test results, so I'll create a dummy test result xml file, with errors %s" % errors
        with open(os.path.join(workspace_test_results_dir, 'dummy.xml'), 'w') as f:
            if errors:
                f.write('<?xml version="1.0" encoding="UTF-8"?><testsuite tests="1" failures="0" time="1" errors="1" name="%s test"> <testcase name="%s rapport" classname="Results" /><testcase classname="%s_class" name="%sFailure"><error type="%sException">%s</error></testcase></testsuite>' % (prefix, prefix, prefix, prefix, prefix, errors))
            else:
                f.write('<?xml version="1.0" encoding="UTF-8"?><testsuite tests="1" failures="0" time="1" errors="0" name="dummy test"> <testcase name="dummy rapport" classname="Results" /></testsuite>')


def copy_static_analysis_results(buildspace_test_results_dir, workspace_test_results_dir):
    """
    Copy static analysis results from buildspace into workspace.

    @param buildspace_test_results_dir: path where the test results are stored
    @type  buildspace_test_results_dir: str
    @param workspace_test_results_dir: path the test results will copied into
    @type  workspace_test_results_dir: str
    """
    # Copy all static analysis results (all xml files)
    for root, dirnames, filenames in os.walk(os.path.join(buildspace_test_results_dir)):
        for filename in fnmatch.filter(filenames, '*.xml'):
            call("cp %s %s/" % (os.path.join(root, filename), workspace_test_results_dir))


def get_ros_env(setup_file):
    """
    Source the setup_file and return a dictionary of env vars

    @param setup_file: path of file to source
    @type  setup_file: str

    @return type: dict
    """
    res = os.environ
    print "Retrieve the ROS build environment by sourcing %s" % setup_file
    command = ['bash', '-c', 'source %s && env' % setup_file]
    proc = subprocess.Popen(command, stdout=subprocess.PIPE)
    for line in proc.stdout:
        (key, _, value) = line.partition("=")
        res[key] = value.split('\n')[0]
    proc.communicate()
    if proc.returncode != 0:
        msg = "Failed to source %s" % setup_file
        print r"/!\  %s" % msg
        raise BuildException(msg)
    return res


def call_with_list(command, envir=None, verbose=True):
    """
    Call a shell command as list.

    @param command: the command to call
    @type  command: list
    @param envir: mapping of env variables
    @type  envir: dict
    @param verbose: print all
    @type  verbose: bool

    @return param: command output
    @return type: str

    @raise type: BuildException
    """
    print "Executing command '%s'" % ' '.join(command)
    helper = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, close_fds=True, env=envir)
    if verbose:
        for line in iter(helper.stdout.readline, b''):
            print line,

    out, err = helper.communicate()
    if helper.returncode != 0:
        msg = "Failed to execute command '%s'" % command
        print r"/!\  %s" % msg
        print out, err
        raise BuildException(msg)

    # print output if verbose
    if verbose:
        print out
    
    # print error always
    print err
    
    return out, err


def call(command, envir=None, verbose=True):
    """
    Call a shell command.

    @param command: the command to call
    @type  command: str
    @param envir: mapping of env variables
    @type  envir: dict
    @param verbose: print all
    @type  verbose: bool

    @return param: command output
    @return type: str
    """
    return call_with_list(command.split(' '), envir, verbose)


def output(message, decoration='*', blankline='a'):
    """
    Output a message in a prominent way

    @param message: text to print
    @type  message: str
    @param decoration: character in which to message will be enclosed
    @type  decoration: str
    @param blankline: position of a blank line, a=above and b=below
    @type  blankline: str
    """

    if 'a' in blankline:
        print ''
    if decoration != '':
        print len(message) * decoration
    print message
    if decoration != '':
        print len(message) * decoration
    if 'b' in blankline:
        print ''


def get_catkin_stack_deps(xml_path):
    import xml.etree.ElementTree as ET
    tree = ET.parse(xml_path)
    root = tree.getroot()
    return list(set([d.text for d in root.findall('depends')]
                    + [d.text for d in root.findall('build_depends')]
                    + [d.text for d in root.findall('run_depends')]))


def get_nonlocal_dependencies(catkin_packages, stacks, manifest_packages, build_depends=True, test_depends=True):
    append_pymodules_if_needed()
    from catkin_pkg import packages
    import rospkg

    depends = []
    #First, we build the catkin deps
    for name, path in catkin_packages.iteritems():
        pkg_info = packages.parse_package(path)
        if build_depends:
            depends.extend([d.name
                            for d in pkg_info.buildtool_depends + pkg_info.build_depends
                            if not d.name in catkin_packages and not d.name in depends])
        if test_depends:
            depends.extend([d.name
                            for d in pkg_info.test_depends + pkg_info.run_depends
                            if not d.name in catkin_packages and not d.name in depends])

    #Next, we build the manifest deps for stacks
    for name, path in stacks.iteritems():
        stack_manifest = rospkg.parse_manifest_file(path, rospkg.STACK_FILE)
        if stack_manifest.is_catkin:
            depends.extend(get_catkin_stack_deps(os.path.join(path, 'stack.xml')))
        else:
            depends.extend([d.name
                            for d in stack_manifest.depends + stack_manifest.rosdeps
                            if not d.name in catkin_packages
                            and not d.name in stacks
                            and not d.name in depends])

    #Next, we build manifest deps for packages
    for name, path in manifest_packages.iteritems():
        pkg_manifest = rospkg.parse_manifest_file(path, rospkg.MANIFEST_FILE)
        depends.extend([d.name
                        for d in pkg_manifest.depends + pkg_manifest.rosdeps
                        if not d.name in catkin_packages
                        and not d.name in stacks
                        and not d.name in manifest_packages
                        and not d.name in depends])

    return depends


def get_dry_packages(source_folder):
    """
    Get all dry stacks and packages in source folder

    @param source_folder: path to search
    @type  source_folder: str

    @return param: list of stacks and list of packages
    @return type: tuple
    """

    catkin_packages, stacks, manifest_packages = get_all_packages(source_folder)
    return (stacks, manifest_packages)


def get_all_packages(source_folder, filter_=True):
    """
    Get all packages (wet and dry)

    @param source_folder: path to search
    @type  source_folder: str
    @param filter_: filter out dry stacks/packages if also wet
    @type filter_: bool

    @return param: list of catkin packages, dry stacks and dry packages
    @return type: tuple
    """
    import rospkg

    catkin_packages = {}
    stacks = {}
    manifest_packages = {}

    rospkg.list_by_path('package.xml', source_folder, catkin_packages)
    rospkg.list_by_path('stack.xml', source_folder, stacks)
    rospkg.list_by_path('manifest.xml', source_folder, manifest_packages)

    # if repo has package.xml and stack.xml/manifest.xml
    # remove stack/manifest entries
    if filter_:
        for name, path in catkin_packages.iteritems():
            if name in stacks:
                del stacks[name]
            if name in manifest_packages:
                del manifest_packages[name]

    return (catkin_packages, stacks, manifest_packages)


def build_local_dependency_graph(catkin_packages, manifest_packages):
    append_pymodules_if_needed()
    from catkin_pkg import packages
    import rospkg

    depends = {}
    #First, we build the catkin dep tree
    for name, path in catkin_packages.iteritems():
        depends[name] = []
        pkg_info = packages.parse_package(path)
        for dep in pkg_info.buildtool_depends + pkg_info.build_depends + pkg_info.test_depends + pkg_info.run_depends:
            if dep.name in catkin_packages and dep.name != name:
                depends[name].append(dep.name)

    #Next, we build the manifest dep tree
    for name, path in manifest_packages.iteritems():
        manifest = rospkg.parse_manifest_file(path, rospkg.MANIFEST_FILE)
        depends[name] = []
        for dep in manifest.depends + manifest.rosdeps:
            if (dep.name in catkin_packages or dep.name in manifest_packages) and dep.name != name:
                depends[name].append(str(dep.name))

    return depends


def reorder_paths(order, packages, paths):
    """
    Reorder paths
    """
    #we want to make sure that we can still associate packages with paths
    new_paths = []
    for package in order:
        old_index = [i for i, name in enumerate(packages) if package == name][0]
        new_paths.append(paths[old_index])

    return order, new_paths


def get_dependency_build_order(depends):
    """
    Get order how to build dependencies

    @param depends: packages and their dependencies
    @type  depends: dict
    """
    import networkx as nx
    graph = nx.DiGraph()

    for name, deps in depends.iteritems():
        graph.add_node(name)
        graph.add_edges_from([(name, d) for d in deps])

    order = nx.topological_sort(graph)
    order.reverse()

    return order


def get_dependencies(source_folder, build_depends=True, test_depends=True):
    """
    Get the dependencies of all packages in the given folder.

    @param source_folder: path of folder to search packages in
    @type  source_folder: str
    @param build_depends: get build dependencies
    @type  build_depends: bool
    @param test_depends: get test dependencies
    @type  test_depends: bool

    @return param: build and/or test dependencies
    @return type:  list
    """
    print "Get the dependencies of source folder %s" % source_folder
    append_pymodules_if_needed()
    from catkin_pkg import packages
    pkgs = packages.find_packages(source_folder)
    local_packages = [p.name for p in pkgs.values()]
    if len(pkgs) > 0:
        print "In folder %s, found packages %s" % (source_folder, ', '.join(local_packages))
    else:
        raise BuildException("Found no packages in folder %s. Are you sure your packages have a packages.xml file?" % source_folder)

    depends = []
    for name, pkg in pkgs.iteritems():
        if build_depends:
            for dep in pkg.build_depends + pkg.buildtool_depends:
                if not dep.name in depends and not dep.name in local_packages:
                    depends.append(dep.name)
        if test_depends:
            for dep in pkg.test_depends + pkg.run_depends:
                if not dep.name in depends and not dep.name in local_packages:
                    depends.append(dep.name)

    return depends


def get_buildpipeline_configs(server_name, user_name, config_repo=None):
    """
    Get buildpipeline configuration

    :param server_name: name of Jenkins master, ``str``
    :param user_name: name of user, ``str``
    :param config_repo: address of configs repository (optional), ``st``

    :returns: return :dict: with configurations
    :raises: :exec:`Exception`
    """
    if config_repo:
        try:
            pipeconfig_url = config_repo.replace(".git", "")
            pipeconfig_url = pipeconfig_url.replace("https://github.com/", "https://raw.github.com/")
            pipeconfig_url = pipeconfig_url.replace("git://github.com/", "https://raw.github.com/")
            pipeconfig_url = pipeconfig_url.replace("git@github.com:", "https://raw.github.com/")
            pipeconfig_url = pipeconfig_url + "/master/%s/%s/pipeline_config.yaml" % (server_name, user_name)
            print "Parsing buildpipeline configuration file for %s stored at:\n%s" % (user_name, pipeconfig_url)

            with contextlib.closing(urllib2.urlopen(pipeconfig_url)) as f:
                bpl_configs = yaml.load(f.read())
        except Exception as ex:
            print "While downloading and parsing the buildpipeline configuration \
                   file from\n%s\nthe following error occured:\n%s" % (pipeconfig_url, ex)
            raise ex

    else:
        print "Parsing buildpipeline configuration file for %s stored at:\n%s" % (user_name, server_name)
        try:
            client = paramiko.SSHClient()
            client.load_system_host_keys()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(hostname=server_name, username="jenkins", key_filename=os.path.expanduser("~/.ssh/id_rsa"))
            sftp = client.open_sftp()
            fileObject = sftp.file("jenkins-config/jenkins_config/" + server_name + "/" + user_name + "/pipeline_config.yaml", 'rb')
            bpl_configs = yaml.load(fileObject.read())
        except Exception as ex:
            print "While downloading and parsing the buildpipeline configuration \
                file from\n%s\nthe following error occured:\n%s" % (server_name, ex)
            raise ex

    return bpl_configs


class BuildException(Exception):
    """
    Build specific exception
    """

    def __init__(self, msg):
        print msg
        self.msg = msg
        super(BuildException, self).__init__(msg)
