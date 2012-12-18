#!/usr/bin/env python

import sys
import os
import subprocess
import paramiko

PLATFORMS = {'electric': ['lucid', 'maverick', 'natty', 'oneiric'],
             'fuerte': ['lucid', 'oneiric', 'precise'],
             'groovy': ['oneiric', 'precise', 'quantal']
             }
ARCH = ['i386', 'amd64']


def main():
    '''TODO'''

    errors = []

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect('jenkins-test-server', username='jenkins')

    print "\nGet existent chroot tarballs"
    existent_tarballs = get_existent_tarballs(ssh)
    for tar in existent_tarballs:
        print " ", tar

    print "\nCalculate necessary chroot envs"
    basic_tarballs, extended_tarballs = get_tarball_lists()
    print "Basic tarballs:"
    for basic in basic_tarballs:
        print " ", basic
    print "Extended tarballs:"
    for extend in extended_tarballs:
        print " ", extend

    sys.stdout.flush()

    for basic in basic_tarballs:
        print "\nvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv"
        print "Set up basic chroot %s" % basic
        local_abs_basic = os.path.join(os.getenv("WORKSPACE"), basic)
        result = basic_tarball(ssh, basic, local_abs_basic, extended_tarballs, existent_tarballs)
        if result != []:
            errors += result

        call('sudo rm %s' % local_abs_basic)
        print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n"

    if existent_tarballs != []:
        print "\n,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,"
        print "Not all chroot tarballs were updated:"
        for tar in existent_tarballs:
            print " ", tar
        print "'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''\n"

    return errors


def basic_tarball(ssh, basic, local_abs_basic, extended_tarballs, existent_tarballs):
    remote_abs_basic = '/home-local/jenkins/chroot_tarballs/%s' % basic

    # get tarball parameter
    tarball_params = get_tarball_params(basic)
    print "tarball parameter: ", tarball_params

    sys.stdout.flush()

    if basic in existent_tarballs:
        print "Update %s" % basic
        existent_tarballs.remove(basic)

        try:
            # copy tarball to slave for update process
            get_tarball(ssh, basic, remote_abs_basic, local_abs_basic)

            # update tarball
            call("./pbuilder_calls.sh update %s" % local_abs_basic)

        except Exception as ex:
            return ["%s: %s" % (basic, ex.msg)]

    else:
        print "Create %s" % basic

        try:
            # create tarball on slave
            call("./pbuilder_calls.sh create %s %s %s"
                 % (local_abs_basic, tarball_params['ubuntu_distro'],
                    tarball_params['arch']))

        except Exception as ex:
            return ["%s: %s" % (basic, ex.msg)]

    sys.stdout.flush()

    try:
        put_tarball(ssh, basic, local_abs_basic, remote_abs_basic)
    except Exception as ex:
        return ["%s: %s" % (basic, ex.msg)]

    sys.stdout.flush()

    # get all tarballs that extend the basic tarball
    extending_tarballs = [extend for extend in extended_tarballs if basic in extend]
    print "Extending tarballs: ", extending_tarballs

    sys.stdout.flush()

    errors = []

    for extend in extending_tarballs:
        print "\nvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv"
        print "Extend basic chroot %s to %s" % (basic, extend)
        local_abs_extend = os.path.join(os.getenv("WORKSPACE"), extend)
        result = extend_tarball(ssh, basic, extend, local_abs_extend, existent_tarballs)
        if result != []:
            errors += result
        call('sudo rm %s' % local_abs_extend)
        print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n"

    sys.stdout.flush()

    return errors


def extend_tarball(ssh, basic, extend, local_abs_extend, existent_tarballs):
    local_abs_basic = os.path.join(os.getenv("WORKSPACE"), basic)
    remote_abs_extend = '/home-local/jenkins/chroot_tarballs/%s' % extend

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
            #call("./pbuilder_calls.sh update %s" % local_abs_extend)

            call("./pbuilder_calls.sh execute %s install_basics.sh %s %s"
                 % (local_abs_extend, tarball_params['ubuntu_distro'], tarball_params['ros_distro']))

        except Exception as ex:
            return ["%s: %s" % (extend, ex.msg)]

    else:
        print "Create %s" % basic
        try:
            call("cp %s %s" % (local_abs_basic, local_abs_extend))

            call("./pbuilder_calls.sh execute %s install_basics.sh %s %s"
                 % (local_abs_extend, tarball_params['ubuntu_distro'], tarball_params['ros_distro']))

        except Exception as ex:
            return ["%s: %s" % (extend, ex.msg)]

    sys.stdout.flush()

    try:
        put_tarball(ssh, extend, local_abs_extend, remote_abs_extend)
    except Exception as ex:
        return ["%s: %s" % (extend, ex.msg)]

    return []


def put_tarball(ssh, tar_name, from_location, to_location):
    print "Copying %s to %s" % (tar_name, ssh.get_host_keys().keys()[0])
    try:
        ftp = ssh.open_sftp()
        ftp.put(from_location,
                to_location)
    finally:
        ftp.close()
    print "Copied succesfully %s to %s" % (tar_name, ssh.get_host_keys().keys()[0])


def get_tarball(ssh, tar_name, from_location, to_location):
    print "Copying %s from %s" % (tar_name, ssh.get_host_keys().keys()[0])
    try:
        ftp = ssh.open_sftp()
        ftp.get(from_location,
                to_location)
    finally:
        ftp.close()
    print "Copied succesfully %s from %s" % (tar_name, ssh.get_host_keys().keys()[0])


def get_existent_tarballs(ssh):
    file_list = []
    stdin, stdout, stderr = ssh.exec_command("ls -1 chroot_tarballs")
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


def get_tarball_lists():
    basic_tarballs = []
    extended_tarballs = []
    for ros_distro, ubuntu_list in PLATFORMS.iteritems():
        for ubuntu_distro in ubuntu_list:
            for arch in ARCH:
                if '__'.join([ubuntu_distro, arch]) not in basic_tarballs:
                    basic_tarballs.append('__'.join([ubuntu_distro, arch]))
                extended_tarballs.append('__'.join([ubuntu_distro, arch, ros_distro]))
    return sorted(basic_tarballs), sorted(extended_tarballs)


def call_with_list(command, envir=None, verbose=True):
    print "\n********************************************************************"
    print "Executing command '%s'" % ' '.join(command)
    print "********************************************************************"
    helper = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, close_fds=True, env=envir)
    res, err = helper.communicate()
    if verbose:
        print str(res)
    print str(err)
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
        print ex.msg

    except Exception as ex:
        print "\n,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,"
        print "Update script failed. Check console output for details."
        print "'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''\n"
        raise ex
