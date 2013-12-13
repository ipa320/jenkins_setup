#!/usr/bin/env python

"""
This module provides the classes CobPipe, CobPipeDependencyRepo, CobPipeRepo
and CobPipeException. Those can be used to instance a CobPipe object which
includes all pipeline configurations of given pipline_config.yaml, locally
stored or on an GitHub account.
"""

import yaml

from jenkins_setup import common


class CobPipe(object):
    """
    Pipeline configuration class
    """

    def __init__(self):
        self.user_name = ""
        self.server_name = ""
        self.email = ""
        self.committer_email_enabled = ""
        self.repositories = {}
        self.pipeline_repos_owner = ""

    def load_config_from_dict(self, pipeline_config):
        """
        Set up a pipeline object derived from the given dictionary

        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        self.user_name = pipeline_config['user_name']
        self.server_name = pipeline_config['server_name']
        self.email = pipeline_config['email']
        self.committer_email_enabled = pipeline_config['committer_email_enabled']

        for repo_name, data in pipeline_config['repositories'].iteritems():
            repo = CobPipeRepo(repo_name, data)
            self.repositories[repo_name] = repo

    def load_config_from_url(self, pipeline_repos_owner, server_name, user_name):
        """
        Get the buildpipeline configuration by the given server and user name
        and set up the pipeline object

        @param pipeline_repos_owner: address of config repo
        @type  pipeline_repos_owner: str
        @param server_name: name of server
        @type  server_name: str
        @param user_name: name of user
        @type  user_name: str
        """

        pipeline_config = common.get_buildpipeline_configs(server_name, user_name)
        self.load_config_from_dict(pipeline_config)
        self.pipeline_repos_owner = pipeline_repos_owner

    def get_jobs_to_create(self):
        """
        Get a dict of all job types to create and the repositories which will
        use them

        @return type: dict
        """

        job_type_dict = {}
        for repo in self.repositories.keys():
            for job in self.repositories[repo].jobs:
                if job in job_type_dict:
                    job_type_dict[job].append(repo)
                else:
                    job_type_dict[job] = [repo]

        return job_type_dict

    def get_custom_dependencies(self, polled_only=False):
        """
        Get all dependencies defined in the pipeline and their corresponding
        repositories

        @param polled_only: if set only polled dependencies will be considered
        @type  polled_only: bool
        @return type: dict
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


class CobPipeDependencyRepo(object):
    """
    Cob pipeline dependency repository class
    """

    def __init__(self, name, data):
        """
        @param name: repository name
        @type  name: str
        @param data: repository and dependency information
        @type  data: dict
        """

        if name is None or name == "":
            raise CobPipeException("No Name given!")
        else:
            self.name = name
        self.type = data['type']
        self.url = data['url']
        self.version = None
        if 'version' in data:
            self.version = data['version']

        self.poll = None
        if 'poll' in data:
            self.poll = data['poll']

        self.test = None
        if 'test' in data:
            self.test = data['test']

    def get_rosinstall(self):
        """
        Gets the rosinstall file entry for the repository object

        @returns type: str
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


class CobPipeRepo(CobPipeDependencyRepo):
    """
    Cob pipeline repository class
    """

    def __init__(self, name, data):
        """
        @param name: repository name
        @type  name: str
        @param data: repository and dependency information
        @type  data: dict
        """

        super(CobPipeRepo, self).__init__(name, data)

        self.poll = True  # first level repos are always polled
        self.ros_distro = data['ros_distro']
        self.prio_ubuntu_distro = data['prio_ubuntu_distro']
        self.prio_arch = data['prio_arch']

        self.regular_matrix = {}
        if data['regular_matrix']:
            self.regular_matrix = data['regular_matrix']

        self.dependencies = {}
        if data['dependencies']:
            for dep_name, dep_data in data['dependencies'].iteritems():
                dep = CobPipeDependencyRepo(dep_name, dep_data)
                self.dependencies[dep_name] = dep

        self.jobs = []
        if data['jobs']:
            self.jobs = data['jobs']

        self.robots = []
        if data['robots']:
            self.robots = data['robots']

        # catch some errors
        if self.robots != [] and 'hardware_build' not in self.jobs:
            raise CobPipeException("Found robot to build on, but job is missing")
        if ('automatic_hw_test' in self.jobs or 'interactive_hw_test' in self.jobs) and self.robots == []:
            raise CobPipeException("Hardware tests defined but no robot to run them on")
        if self.regular_matrix != {} and 'regular_build' not in self.jobs:
            raise CobPipeException("Configuration for regular build found, but no regular build job")


class CobPipeException(Exception):

    """
    CobPipeline specific exception
    """

    def __init__(self, msg):
        print msg
        self.msg = msg
        super(CobPipeException, self).__init__(msg)
