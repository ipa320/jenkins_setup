#!/usr/bin/env python

import urllib2
import yaml


class Cob_Distro(object):
    """
    TODO
    """

    def __init__(self, name, repos_dict=None):
        """
        TODO
        """

        self.repositories = {}

        if not repos_dict:
            self.url = 'https://raw.github.com/fmw-jk/jenkins_setup/master/releases/cob_%s.yaml' % name  # TODO change to ipa320
            self.release_file = urllib2.urlopen(self.url)
            self.repos_dict = yaml.load(self.release_file.read())['repositories']
        else:
            self.repos_dict = repos_dict

        for repo_name, data in self.repos_dict.iteritems():
            repo = Cob_Distro_Repo(repo_name, data)
            self.repositories[repo_name] = repo


class Cob_Distro_Repo(object):
    """
    Object containing repository information
    """
    def __init__(self, name, data):
        """
        :param name: repository name, ``str``
        :param data: repository information, e.g. from a rosdistro file, ``dict``
        """

        self.name = name
        self.type = data['type']
        self.url = data['url']
        self.version = None
        if 'version' in data:
            self.version = data['version']
        self.poll = None
        if 'poll' in data:
            self.poll = data['poll']
        self.dep = None
        if 'dep' in data:
            self.dep = data['dep']
        self.private = None
        if 'private' in data:
            self.private = data['private']

    def get_rosinstall(self):
        """
        Generate a rosinstall file entry with the stored information.

        :returns: return rosinstall file entry, ``str``
        """

        if self.version:
            return yaml.dump([{self.type: {'local-name': self.name,
                                           'uri': '%s' % self.url,
                                           'version': '%s' % self.version}}],
                             default_style=False)
        else:
            return yaml.dump([{self.type: {'local-name': self.name,
                                           'uri': '%s' % self.url}}],
                             default_style=False)
