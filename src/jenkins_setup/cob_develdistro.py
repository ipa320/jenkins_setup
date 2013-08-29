#!/usr/bin/env python

import urllib2
import yaml
from rosdistro import develdistro


class CobDevelDistro(develdistro.DevelDistro):
    """
    Cob devel distro class
    """

    def __init__(self, name, url=None):
        """
        Initializes the distro object and sets up all repositories derived
        from the given distro yaml file

        :param name: name of distro, ``str``
        :param url: optional url where to get the distro file from, ``str``
        """

        # TODO call super ???

        self.repositories = {}

        if url:
            self.url = url
            self.release_file = urllib2.urlopen(self.url)
            self.repos_dict = yaml.load(self.release_file.read())['repositories']
        else:
            self.url = 'https://raw.github.com/ipa320/jenkins_setup/master/releases/cob_%s-devel.yaml' % name
            self.release_file = urllib2.urlopen(self.url)
            self.repos_dict = yaml.load(self.release_file.read())['repositories']

        for repo_name, data in self.repos_dict.iteritems():
            repo = develdistro.DevelDistroRepo(repo_name, data)
            self.repositories[repo_name] = repo
