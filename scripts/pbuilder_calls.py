#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import subprocess
from core.CommandDispatcher2 import CommandDispatcher
from core.CommandDispatcher2 import STDIOTYPE


def fastStdFeedback(data):
    foo = data
    while len(foo):
        print len(foo), foo.pop()
        sys.stdout.flush()

def fireCommand(command, timeout=180):
    print 'executing command..'
    sys.stdout.flush()

    stdIOType = [STDIOTYPE.STDOUT, STDIOTYPE.STDERR]
    cd = CommandDispatcher('/bin/bash', stdIOType, dataCallbackFnc=fastStdFeedback, timeout=timeout, commandSleepTime=0.2, syncThreads=True)
    cd.register(fastStdFeedback)
    cd.sendCmd(' '.join(command))
    cd.join()

    print '-+-'*20
    print 'command ending....'
    print '-+-'*20


if __name__ =='__main__':
    print 'Script arguments are:', sys.argv
    print
    print 'os.environ', os.environ
    print

    job = sys.argv[1]

    if job == 'create':
        basetgz = sys.argv[2]
        ubuntu_distro = sys.argv[3]
        arch = sys.argv[4]

        print 'basetgz', basetgz
        print 'ubuntu_distro', ubuntu_distro
        print 'arch', arch

        command = ['sudo', 'pbuilder', '--create', '--basetgz', basetgz,
                   '--distribution', ubuntu_distro, '--architecture', arch,
                   '--debootstrapopts', '--variant=buildd', '--components',
                   '"main universe multiverse"', '--debootstrapopts',
                   '--keyring=/etc/apt/trusted.gpg', '--timeout', '5m']

        print 'command will be:',  ' '.join(command)

        fireCommand(command)

        #print subprocess.check_output(command, close_fds=True, env=os.environ)


    elif job == 'update':
        basetgz = sys.argv[2]

        print 'basetgz', basetgz

        command = ['sudo', 'pbuilder', '--update', '--basetgz', basetgz]

        print 'command will be:',  ' '.join(command)

        fireCommand(command)


    elif job == 'execute':
        basetgz = sys.argv[2]
        script_name = sys.argv[3]
        script_arguments = sys.argv[4:]

        print 'basetgz', basetgz
        print 'script_name', script_name
        print 'script_arguments', script_arguments


        command = ['sudo', 'pbuilder', '--execute', '--basetgz', basetgz,
                   '--save-after-exec', '--', script_name, ' '.join(script_arguments)]

        print 'command will be:',  ' '.join(command)
        fireCommand(command)

    sys.exit()
