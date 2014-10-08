#!/usr/bin/env python

import sys
import os
import subprocess
import paramiko
import urllib2
import yaml
import optparse

ARCH = ['i386', 'amd64']


def main():
    """
    Set up and update all chroot tarballs
    """

    errors = []

    # parse options
    parser = optparse. OptionParser()
    (options, args) = parser.parse_args()

    if len(args) < 4:
        print "Usage: %s tarball_location_ssh_address target_yaml_url ubuntu_distro architecture [apt_cacher_proxy_address]" % (sys.argv[0])
        sys.exit()

    print args

    tarball_location_ssh_address = args[0]
    target_platforms_url = args[1]

    tarball_host = tarball_location_ssh_address.split(':')[0].split('@')[1]
    tarball_host_username = tarball_location_ssh_address.split('@')[0]
    tarball_dir = tarball_location_ssh_address.split(':')[1]

    try:
        f = urllib2.urlopen(target_platforms_url)
        platforms = yaml.load(f)
    except Exception as ex:
        print "While downloading and parsing target platforms file from\n%s\n \
               the following error occured:\n%s" % (target_platforms_url, ex)
        raise ex

    # check if given ubuntu distro and arch is supported
    ubuntu_distro = args[2]
    supported_ubuntu_distros = []
    for ros_distro_dict in platforms:
        for ros_distro, ubuntu_distro_list in ros_distro_dict.iteritems():
            if ros_distro != "backports":
                for supported in ubuntu_distro_list:
                    supported_ubuntu_distros.append(supported)
    if ubuntu_distro not in supported_ubuntu_distros:
        print "Ubuntu distro %s not supported! Supported Ubuntu distros :" % ', '.join(sorted(supported_ubuntu_distros))
        sys.exit()
    arch = args[3]
    if arch not in ARCH:
        print "Architecture %s not supported! Supported architectures: %s" % (arch, ', '.join(ARCH))
        sys.exit()

    apt_cacher_proxy = ''
    if len(args) == 5:
        apt_cacher_proxy = args[4]

    # set up ssh object
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(tarball_host, username=tarball_host_username)

    # get home folder
    if tarball_dir.startswith('~/'):
        tarball_dir = tarball_dir.replace('~/', '')
    if not tarball_dir.startswith('/'):
        tarball_dir = os.path.join(get_home_folder(ssh), tarball_dir)

    print "\nGet existent chroot tarballs"
    existent_tarballs = get_existent_tarballs(ssh, tarball_dir)
    for tar in existent_tarballs:
        print " ", tar

    print "\nCalculate chroot envs to set up / update"
    basic_tarball, extended_tarballs = get_tarball_names(platforms, ubuntu_distro, arch)
    print "Basic tarball:"
    print " ", basic_tarball
    print "Extended tarballs: \n %s" % '\n '.join(extended_tarballs)

    sys.stdout.flush()

    print "\nvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv"
    print "Set up basic chroot %s" % basic_tarball
    result = process_basic_tarball(ssh, basic_tarball, os.getenv("WORKSPACE"),
                                   tarball_dir, extended_tarballs, existent_tarballs,
                                   apt_cacher_proxy)
    if result != []:
        errors += result

    call('sudo rm %s' % os.path.join(os.getenv("WORKSPACE"), basic_tarball))
    print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n"

    if existent_tarballs != []:
        print "\n,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,"
        print "Not all chroot tarballs were updated:"
        for tar in existent_tarballs:
            print " ", tar
        print "'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''\n"

    return errors


def process_basic_tarball(ssh, basic, local_abs, remote_abs, extended_tarballs, existent_tarballs, apt_cacher_proxy):
    local_abs_basic = os.path.join(local_abs, basic)
    remote_abs_basic = os.path.join(remote_abs, basic)

    sys.stdout.flush()

    # get tarball parameter
    tarball_params = get_tarball_params(basic)
    print "tarball parameter: ", tarball_params

    sys.stdout.flush()

    if basic in existent_tarballs:
#        print "Update %s" % basic
        existent_tarballs.remove(basic)

#        try:
#            # copy tarball to slave for update process
#            get_tarball(ssh, basic, remote_abs_basic, local_abs_basic)
#
#            # update tarball
#            call("./pbuilder_calls.py update %s" % local_abs_basic)
#
#        except Exception as ex:
#            return ["%s: %s" % (basic, ex)]
#
#    else:

    sys.stdout.flush()
    print "Create %s" % basic

    sys.stdout.flush()
    try:
        # create tarball on slave
        call("./pbuilder_calls.py create %s %s %s"
             % (local_abs_basic, tarball_params['ubuntu_distro'],
                tarball_params['arch']))

    except Exception as ex:
        return ["%s: %s" % (basic, ex)]

    sys.stdout.flush()

    try:
        put_tarball(ssh, basic, local_abs_basic, remote_abs_basic)
    except Exception as ex:
        return ["%s: %s" % (basic, ex)]

    sys.stdout.flush()

    # get all tarballs that extend the basic tarball
    extending_tarballs = [extend for extend in extended_tarballs if basic in extend]
    print "Extending tarballs: ", extending_tarballs

    sys.stdout.flush()

    errors = []

    for extend in extending_tarballs:
        print "\nvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv"
        print "Extend basic chroot %s to %s" % (basic, extend)
        local_abs_extend = os.path.join(local_abs, extend)
        remote_abs_extend = os.path.join(remote_abs, extend)
        result = process_extend_tarball(ssh, basic, local_abs_basic, extend,
                                        local_abs_extend, remote_abs_extend,
                                        existent_tarballs, apt_cacher_proxy)
        if result != []:
            errors += result
        call('sudo rm %s' % local_abs_extend)
        print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n"

    sys.stdout.flush()

    return errors


def process_extend_tarball(ssh, basic, local_abs_basic, extend, local_abs_extend,
                           remote_abs_extend, existent_tarballs, apt_cacher_proxy):
    # get tarball parameter
    tarball_params = get_tarball_params(extend)

    sys.stdout.flush()

    if extend in existent_tarballs:
        print "Update %s" % extend
        existent_tarballs.remove(extend)

        try:
            # copy tarball to executing slave
            get_tarball(ssh, extend, remote_abs_extend, local_abs_extend)

            # update tarball
            #call("./pbuilder_calls.py update %s" % local_abs_extend)

            call("./pbuilder_calls.py execute %s install_basics.sh %s %s %s"
                 % (local_abs_extend, tarball_params['ubuntu_distro'],
                    tarball_params['ros_distro'], apt_cacher_proxy))

        except Exception as ex:
            return ["%s: %s" % (extend, ex)]

    else:
        print "Create %s" % basic
        try:
            call("cp %s %s" % (local_abs_basic, local_abs_extend))

            call("./pbuilder_calls.py execute %s install_basics.sh %s %s %s"
                 % (local_abs_extend, tarball_params['ubuntu_distro'],
                    tarball_params['ros_distro'], apt_cacher_proxy))

        except Exception as ex:
            return ["%s: %s" % (extend, ex)]

    sys.stdout.flush()

    try:
        put_tarball(ssh, extend, local_abs_extend, remote_abs_extend)
    except Exception as ex:
        return ["%s: %s" % (extend, ex)]

    return []


def put_tarball(ssh, tar_name, from_location, to_location):
    print "Copying %s to %s" % (tar_name, ssh.get_host_keys().keys()[0])
    try:
        ftp = ssh.open_sftp()
        ftp.put(from_location,
                to_location)
    except Exception as ex:
        print ex
        raise Exception(ex)
    finally:
        ftp.close()
    print "Copied successfully %s to %s" % (tar_name, ssh.get_host_keys().keys()[0])


def get_tarball(ssh, tar_name, from_location, to_location):
    print "Copying %s from %s" % (tar_name, ssh.get_host_keys().keys()[0])
    try:
        ftp = ssh.open_sftp()
        ftp.get(from_location,
                to_location)
    except Exception as ex:
        print ex
        raise Exception(ex)
    finally:
        ftp.close()
    print "Copied successfully %s from %s" % (tar_name, ssh.get_host_keys().keys()[0])


def get_home_folder(ssh):
    stdin, stdout, stderr = ssh.exec_command("pwd")
    return stdout.readline().replace('\n', '')


def get_existent_tarballs(ssh, path):
    file_list = []
    stdin, stdout, stderr = ssh.exec_command("ls -1 %s" % path)
    for line in stdout.readlines():
        line = line.replace('\n', '')
        file_list.append(line)
    return file_list


def get_tarball_params(name):
    params_dict = {}
    name_split = name.split('__')
    if len(name_split) == 2:
        params_dict['ubuntu_distro'], params_dict['arch'] = name_split
        params_dict['ros_distro'] = None
    elif len(name_split) == 3:
        params_dict['ubuntu_distro'], params_dict['arch'], params_dict['ros_distro'] = name_split
    else:
        raise BuildException('Invalid tarball name')
    return params_dict


def get_tarball_names(platforms, ubuntu_distro, arch):
    basic_tarball = '__'.join([ubuntu_distro, arch])
    extended_tarballs = []
    for ros_distro_dict in platforms:
        for ros_distro, ubuntu_distro_list in ros_distro_dict.iteritems():
            if ubuntu_distro in ubuntu_distro_list and ros_distro != "backports":
                extended_tarballs.append('__'.join([ubuntu_distro, arch,
                                                    ros_distro]))
    return basic_tarball, sorted(extended_tarballs)


def call_with_list(command, envir=None, verbose=True):
    print "\n********************************************************************"
    print "Executing command '%s'" % ' '.join(command)
    print "********************************************************************"

    sys.stdout.flush()

    helper = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, close_fds=True, env=envir)
    res, err = helper.communicate()
    #res = subprocess.check_output(command, close_fds=True, env=envir)
    #err = ''
    if verbose:
        print str(res)
    print str(err)

    sys.stdout.flush()


    if helper.returncode != 0:
        msg = "Failed to execute command '%s'" % command
        print "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        print "/!\  %s" % msg
        print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
        raise BuildException(msg)

    return res


def call(command, envir=None, verbose=True):
    return call_with_list(command.split(' '), envir, verbose)


class BuildException(Exception):
    def __init__(self, msg):
        self.msg = msg


if __name__ == "__main__":
    try:
        result = main()

        if result != []:
            print "\n,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,"
            print "Some errors occured. The following chroot environments couldn't be set up:"
            for error in result:
                print " ", error
            print "For further information please check the output above."
            print "'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''\n"
            sys.exit(1)
        else:
            print "\n,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,"
            print "Chroot tarball update finished cleanly"
            print "'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''\n"

    except BuildException as ex:
        print ex

    except Exception as ex:
        print "\n,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,"
        print "Update script failed. Check console output for details."
        print "'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''\n"
        raise ex
