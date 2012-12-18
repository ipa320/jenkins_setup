#!/usr/bin/env python

import urllib2

#import rosdistro
from rosdistro.rosdistro import *


class CobRosDistro(RosDistro):
    def __init__(self, name, cache_location=None):
        RosDistro.__init__(self, name)
        self.distro_file = CobDistroFile(name)


class CobRosDistroFile(object):

    def __init__(self, name):

        self.packages = {}
        self.repositories = {}

        # parse ros distro file
        self.distro_url = urllib2.urlopen('https://raw.github.com/fmw-jk/jenkins_setup/master/releases/cob_%s.yaml' % name)
        self.distro = yaml.load(self.distro_url.read())['repositories']

        # loop over all repo's
        for repo_name, data in self.distro.iteritems():
            repo = RosRepository(repo_name, data['version'], data['url'])
            self.repositories[repo_name] = repo
            if 'packages' not in data:   # support unary disto's
                data['packages'] = {repo_name: ''}

            # loop over all packages
            for pkg_name in data['packages'].keys():
                pkg = RosPackage(pkg_name, repo)
                repo.packages.append(pkg)
                self.packages[pkg_name] = pkg
