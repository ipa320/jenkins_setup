#!/usr/bin/env python

"""
This module provides the classes CobPipe, CobPipeDependencyRepo, CobPipeRepo
and CobPipeException. Those can be used to instance a CobPipe object which
includes all pipeline configurations of given pipline_config.yaml, locally
stored or on an GitHub account.
"""

import yaml
import paramiko
import os
import sys


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

        self.pipeline_config = pipeline_config
        self.user_name = pipeline_config['user_name']
        self.server_name = pipeline_config['server_name']
        self.email = pipeline_config['email']
        self.committer_email_enabled = pipeline_config['committer_email_enabled']

        for repo_name, data in pipeline_config['repositories'].iteritems():
            repo = CobPipeRepo(repo_name, data)
            self.repositories[repo_name] = repo

    def load_config_from_url(self, pipeline_repos_owner, server_name, user_name, url):
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

        pipeconfig_url = url.replace(".git", "")
        pipeconfig_url = pipeconfig_url.replace("https://github.com/", "https://raw.github.com/")
        pipeconfig_url = pipeconfig_url.replace("git://github.com/", "https://raw.github.com/")
        pipeconfig_url = pipeconfig_url.replace("git@github.com:", "https://raw.github.com/")
        pipeconfig_url = pipeconfig_url + "/master/%s/%s/pipeline_config.yaml" % (server_name, user_name)
        print "Parsing buildpipeline configuration file from github for %s stored at:\n%s" % (user_name, pipeconfig_url)
        with contextlib.closing(urllib2.urlopen(pipeconfig_url)) as f:
            pipeline_config = yaml.load(f.read())
        self.load_config_from_dict(pipeline_config)
        self.pipeline_repos_owner = pipeline_repos_owner

    def load_config_from_sftp(self, pipeline_repos_owner, server_name, user_name):
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

        client = paramiko.SSHClient()
        client.load_system_host_keys()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(hostname=server_name, username="jenkins", key_filename=os.path.expanduser("~/.ssh/id_rsa"))
        sftp = client.open_sftp()
        fileObject = sftp.file("jenkins-config/jenkins_config/" + server_name + "/" + user_name + "/pipeline_config.yaml", 'rb')
        pipeline_config = yaml.load(fileObject.read())
        self.load_config_from_dict(pipeline_config)
        self.pipeline_repos_owner = pipeline_repos_owner

    def load_config_from_file(self, pipeline_repos_owner, server_name, user_name, file_location):
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

        if file_location == None:
            file_location = os.environ['WORKSPACE']
        fileObject = open( file_location + "/jenkins_config/" + server_name + "/" + user_name + "/pipeline_config.yaml", 'rb')
        pipeline_config = yaml.load(fileObject.read())

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

    def split_github_url(self, url):
        """
        splits a github url into user and repository name
        
        :param url: github url
        """
        
        user = url.split(':', 1)[1].split('/', 1)[0]
        name = url.split(':', 1)[1].split('/', 1)[1].split('.git')[0]

        return user, name
        
        
    def split_hg_url(self, url):
        """
        splits a hg url into user and repository name
        
        :param url: hg url
        """
        # example ssh://user@server://path/repo1 
        user = url.split('@')[0].split('//')[1]
        name = url.split('/')[8]

        return user, name

    def create_scm_trigger(self, vcs_type, url, version, jobs_to_trigger = []):
        """
        Create a scm trigger with a name and content

        @return type: string
        @return type: dict
        """    
    
        scm_trigger = {}
        scm_trigger['url'] = url
        scm_trigger['vcs_type'] = vcs_type
        if vcs_type == "git": 
            user, repo_name = self.split_github_url(scm_trigger['url'])
        elif vcs_type == "hg":
            user, repo_name = self.split_hg_url(scm_trigger['url'])
        else:
            print "scm type" + vcs_type + "not supported"
            sys.exit()
        scm_trigger['user'] = user
        scm_trigger['repo'] = repo_name
        scm_trigger['version'] = version
        scm_trigger['jobs_to_trigger'] = jobs_to_trigger
        scm_trigger_name = scm_trigger['vcs_type'] + '__' + scm_trigger['user'] + '__' + scm_trigger['repo'] + '__' + scm_trigger['version']
        return scm_trigger_name, scm_trigger

    def get_scm_triggers(self):
        """
        Get all scm triggers with jobs_to_trigger

        @return type: dict
        """

        pipe_repo_list = self.repositories.keys()
        scm_triggers = {}
        for pipe_repo in pipe_repo_list:
            # always add pipe_repo to scm triggers
            vcs_type = self.pipeline_config['repositories'][pipe_repo]['type']
            scm_trigger_name, scm_trigger = self.create_scm_trigger(vcs_type, self.repositories[pipe_repo].data['url'], self.repositories[pipe_repo].data['version'], [pipe_repo])
            if scm_trigger_name in scm_triggers.keys(): # if dependency is already listed in dependencies: extend the jobs_to_trigger with the current repository
                scm_triggers[scm_trigger_name]['jobs_to_trigger'].append(pipe_repo)

            else: # if not listed in scm_triggers: add a new entry
                scm_triggers[scm_trigger_name] = scm_trigger

            # add dependencies to scm triggers if they are marked with poll=true
            for dependency in self.repositories[pipe_repo].data['dependencies'].keys():
                if self.repositories[pipe_repo].data['dependencies'][dependency]['poll']:
                    vcs_type = self.pipeline_config['repositories'][pipe_repo]['type']
                    scm_trigger_name, scm_trigger = self.create_scm_trigger(vcs_type, self.repositories[pipe_repo].data['dependencies'][dependency]['url'], self.repositories[pipe_repo].data['dependencies'][dependency]['version'], [pipe_repo])
                    if scm_trigger_name in scm_triggers.keys(): # if dependency is already listed in dependencies: extend the jobs_to_trigger with the current repository
                        scm_triggers[scm_trigger_name]['jobs_to_trigger'].append(pipe_repo)
                    else: # if not listed in dependencies: add a new entry
                        scm_triggers[scm_trigger_name] = scm_trigger
        return scm_triggers


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
        self.data = data

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
