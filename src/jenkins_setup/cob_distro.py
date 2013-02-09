#!/usr/bin/env python

import urllib2
import yaml

from jenkins_setup import cob_common


class CobDistro(object):
    """
    Object containing distro information
    """

    def __init__(self, name, url=None):
        """
        Initializes the distro object and sets up all repositories derived
        from the given distro yaml file

        :param name: name of distro, ``str``
        :param url: optional url where to get the distro file from, ``str``
        """

        self.repositories = {}

        if url:
            self.url = url
            self.release_file = urllib2.urlopen(self.url)
            self.repos_dict = yaml.load(self.release_file.read())['repositories']
        else:
            self.url = 'https://raw.github.com/fmw-jk/jenkins_setup/master/releases/cob_%s.yaml' % name  # TODO change to ipa320
            self.release_file = urllib2.urlopen(self.url)
            self.repos_dict = yaml.load(self.release_file.read())['repositories']

        for repo_name, data in self.repos_dict.iteritems():
            repo = CobDistroRepo(repo_name, data)
            self.repositories[repo_name] = repo


class CobDistroPipe(object):
    """
    Object containing buildpipeline information
    """

    def load_config_from_dict(self, repos_dict):
        """
        Sets up a pipeline object derived from the given dictionary

        :param repos_dict: containing all buildpipeline configurations, ``str``
        """

        self.repositories = {}
        self.repos_dict = repos_dict

        for repo_name, data in self.repos_dict.iteritems():
            repo = CobDistroPipeRepo(repo_name, data)
            self.repositories[repo_name] = repo

    def load_config_from_url(self, server_name, user_name):
        """
        Gets the buildpipeline configuration by the given server and user name
        and sets up the pipeline object

        :param server_name: name of server, ``str``
        :param user_name: name of user, ``str``
        """

        buildpipe_conf_dict = cob_common.get_buildpipeline_configs(server_name, user_name)
        self.load_config_from_dict(buildpipe_conf_dict['repositories'])

    def get_custom_dependencies(self, polled_only=False):
        """
        Generates a :dict: with dependencies and corresponding repository.

        :param polled_only: if set only polled dependencies will be considered,
        ``bool``
        :returns: return :dict: with dependencies as keys and the corresponding
        repositories (in a list) as value, ``dict``
        """

        deps = {}
        for repo in self.repositories.keys():
            for dep in self.repositories[repo].dependencies.keys():
                if polled_only:
                    if not self.repositories[repo].dependencies[dep].poll:
                        continue
                if dep in deps:
                    deps[dep].append(repo)
                else:
                    deps[dep] = [repo]

        return deps


class CobDistroRepo(object):
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

    def get_rosinstall(self):
        """
        Generate a rosinstall file entry with the stored information.

        :returns: return rosinstall file entry, ``str``
        """

        if self.version:
            return yaml.dump([{self.type: {'local-name': self.name.split('__')[0],
                                           'uri': '%s' % self.url,
                                           'version': '%s' % self.version}}],
                             default_style=False)
        else:
            return yaml.dump([{self.type: {'local-name': self.name.split('__')[0],
                                           'uri': '%s' % self.url}}],
                             default_style=False)


class CobDistroPipeRepo(CobDistroRepo):
    """
    Object containing repository information and additional information for
    the buildpipeline
    """
    def __init__(self, name, data):
        """
        :param name: repository name, ``str``
        :param data: repository and dependency information from pipeline_config
        file, ``dict``
        """

        super(CobDistroPipeRepo, self).__init__(name, data)

        self.poll = True  # first level repos are always polled
        self.ros_distro = data['ros_distro']
        self.prio_ubuntu_distro = data['prio_ubuntu_distro']
        self.prio_arch = data['prio_arch']

        self.matrix_distro_arch = {}
        if data['matrix_distro_arch']:
            self.matrix_distro_arch = data['matrix_distro_arch']

        self.dependencies = {}
        if data['dependencies']:
            for dep_name, dep_data in data['dependencies'].iteritems():
                dep = CobDistroRepo(dep_name, dep_data)
                self.dependencies[dep_name] = dep
            #self.dependencies = CobDistro('deps', repos_dict=data['dependencies'])

        self.jobs = []
        if data['jobs']:
            self.jobs = data['jobs']

        self.robots = []
        if data['robots']:
            self.robots = data['robots']

        # catch some errors
        if ('bringup' in self.jobs or 'highlevel' in self.jobs) and self.robots == []:
            raise CobDistroException("Hardware tests defined but no robot to run them on")
        if self.matrix_distro_arch != {} and 'normal' not in self.jobs:
            raise CobDistroException("Configuration for normal build found, but no normal build job")


class CobDistroException(Exception):
    def __init__(self, msg):
        print msg
        self.msg = msg
