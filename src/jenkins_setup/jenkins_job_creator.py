#!/usr/bin/env python

import datetime
import socket
import pkg_resources
import yaml
import re

from jenkins_setup import cob_distro


class Jenkins_Job(object):
    """
    Object representation of Jenkins job
    """
    def __init__(self, jenkins_instance, pipeline_config):
        """
        Sets up Jenkins job object
        """

        self.JOB_TYPE_NAMES = {'pipe': 'pipe_starter',
                               'prio': 'prio_build',
                               'normal': 'normal_build',
                               'down': 'downstream_build',
                               'db': 'database_test',
                               'sim': 'simulation_test',
                               'app': 'application_test',
                               'clean': 'cleanup',
                               'bringup': 'bringup_hardware_test',
                               'hilevel': 'highlevel_hardware_test',
                               'release': 'release_params'}

        self.jenkins_instance = jenkins_instance
        self.pipe_conf = pipeline_config

        self.job_config_params = pkg_resources.resource_string('jenkins_setup', 'templates/job_config_params.yaml')
        self.job_config_params = yaml.load(self.job_config_params)
        self.job_config = pkg_resources.resource_string('jenkins_setup', 'templates/job_config.xml')

        self.pipe_inst = cob_distro.Cob_Distro_Pipe()
        self.pipe_inst.load_from_dict(self.pipe_conf['repositories'])

    def schedule_job(self):
        """
        Create new or reconfigure existent job
        """
        if self.jenkins_instance.job_exists(self.job_name):
            try:
                self.jenkins_instance.reconfig_job(self.job_name, self.job_config)
                print "Reconfigured job %s" % self.job_name
                return 'reconfigured'
            except Exception as ex:
                print ex
                return 'reconfiguration failed: %s' % ex
        else:
            try:
                self.jenkins_instance.create_job(self.job_name, self.job_config)
                print "Created job %s" % self.job_name
                return 'created'
            except Exception as ex:
                print ex
                return 'creation failed: %s' % ex

    def create_job(self):
        """
        Gets job specific parameter, sets up the job config and creates job
        on Jenkins instance
        """

        self.get_common_params()

        self.get_job_type_params()

        self.replace_placeholder()
        print self.schedule_job()

    def delete_job(self):
        """
        Deletes the job defined by the job name

        :param job_name: name of the job to delete, ``str`` TODO
        :returns: return message, ``str``
        """

        if self.jenkins_instance.job_exists(self.job_name):
            try:
                self.jenkins_instance.delete_job(self.job_name)
            except Exception as ex:
                return 'deletion failed: %s' % ex
            return 'deleted'
        else:
            return 'not existent'

    def get_common_params(self):
        """
        Gets all parameters which have to be defined for every job type
        """

        self.params = {}

        self.params['USERNAME'] = self.pipe_conf['user_name']
        self.params['EMAIL'] = self.pipe_conf['email']
        self.params['EMAIL_COMMITTER'] = self.pipe_conf['committer']
        #self.params['JOB_TYPE_NAME'] = self.JOB_TYPE_NAMES[self.job_type]
        self.params['SCRIPT'] = self.JOB_TYPE_NAMES[self.job_type]
        self.params['NODE_LABEL'] = self.job_type
        self.params['TIME'] = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d %H:%M')
        self.params['HOSTNAME'] = socket.gethostname()
        self.params['PROJECT'] = 'matrix-project'
        self.params['TRIGGER'] = self.job_config_params['triggers']['none']
        self.params['COMMAND'] = ''
        self.params['VCS'] = self.job_config_params['vcs']['none']
        self.params['MATRIX'] = ''
        self.params['PARAMETERS'] = ''
        self.params['POSTBUILD_TRIGGER'] = ''
        self.params['JOIN_TRIGGER'] = ''
        self.params['PIPELINE_TRIGGER'] = ''
        self.params['GROOVY_POSTBUILD'] = ''
        self.params['PARAMETERIZED_TRIGGER'] = ''

    ###########################################################################
    # helper methods - parameter generation
    ###########################################################################
    def replace_placeholder(self):
        '''
        replace placeholder in template with params
        '''

        for key, value in self.params.iteritems():
            if "@(%s)" % key not in self.job_config:
                raise KeyError("Parameter %s could not be replaced, because it is not existent" % key)
            self.job_config = self.job_config.replace("@(%s)" % key, value)
        not_replaced_keys = re.findall('@\(([A-Z0-9_]+)\)', self.job_config)
        if not_replaced_keys != []:
            raise KeyError("The keys %s were not replaced, because the parameters where missing" % (str(not_replaced_keys)))

    def generate_job_name(self, job_type, suffix=''):
        '''
        Returns a job_name string generated from a job_type plus suffix

        :param job_type: short job type name, ``str``
        :param suffix: will be added to the end of the name, ``str``
        '''

        if suffix != '':
            return '__'.join([self.pipe_conf['user_name'],
                              self.JOB_TYPE_NAMES[job_type],
                              suffix])
        else:
            return '__'.join([self.pipe_conf['user_name'],
                              self.JOB_TYPE_NAMES[job_type]])

    def generate_job_list(self, job_type_list):
        '''
        returns a list of job_names generated from a list of job_types
        '''

        if type(job_type_list) != list:
            raise TypeError("Input type is not type 'list'")
        return [self.generate_job_name(job_type) for job_type in job_type_list]

    def generate_job_list_string(self, job_type_list):
        '''
        returns a string of comma separated job_names generated from a list of job_types
        '''

        return ', '.join(self.generate_job_list(job_type_list))

    def generate_matrix_filter(self, config, negation=False):
        """
        Returns a groovy combination filter

        :param config: couples of names and values which have to be met, ``dict``
        :param negation: negates the resulting filter, ``bool``
        :returns: matrix config, ``str``
        """

        filter = '(%s)' % ' || '.join(['(%s)' % ' &amp;&amp; '.join(['%s == %s' % (key, value)
                                                                     for key, value in i.iteritems()])
                                       for i in config])
        if negation:
            filter = '!' + filter

        return filter

    def generate_matrix_axis(self, axis_name, value_list):
        """
        Returns matrix axis config for given list of values

        :param axis_name: name of axis, ``str``
        :param value_list: list with value, ``list``
        :returns: axis config, ``str``
        """

        if axis_name == '':
            raise Exception('No proper name given')
        if value_list == []:
            raise Exception('No values given')
        axis = self.job_config_params['matrix']['axis'].replace('@(NAME)', axis_name)
        values = ' '.join([self.job_config_params['matrix']['value'].replace('@(VALUE)', value) for value in value_list])
        axis = axis.replace('@(VALUES)', values)

        return axis

    def generate_matrix_param(self, name_value_dict, filter=None):
        """
        Returns matrix config for given dictionary containing names and values

        :param name_value_dict: matrix parameter config, ``dict``
        :param filter: combination filter, ``str``
        :returns: matrix config, ``str``
        """

        axes = ''
        if name_value_dict == {}:
            return ''
        axes += ' '.join([self.generate_matrix_axis(axis_name, axis_values)
                          for axis_name, axis_values in name_value_dict.iteritems()])

        matrix = self.job_config_params['matrix']['basic'].replace('@(AXES)', axes)
        if filter:
            matrix += ' ' + self.job_config_params['matrix']['filter'].replace('@(FILTER)', filter)

        return matrix

    def generate_jointrigger_param(self, job_type_list, unstable_behavior=False):
        """
        Generates config for jointrigger plugin

        :param job_type_list: list with job types (short) to join, ``list``
        :param unstable_behavior: execute join project even if upstream projects
        where unstable, ``bool``

        :returns: configuration of jointrigger plugin, ``str``
        """

        if job_type_list == []:
            return ''
        elif type(unstable_behavior) != bool:
            raise Exception("Behavior argument for unstable job result has to be a boolean")

        jointrigger = self.job_config_params['jointrigger'].replace('@(JOIN_PROJECTS)',
                                                                    self.generate_job_list_string(job_type_list))
        if unstable_behavior:
            jointrigger = jointrigger.replace('@(JOIN_UNSTABLE)', 'true')
        else:
            jointrigger = jointrigger.replace('@(JOIN_UNSTABLE)', 'false')
        return jointrigger

    def generate_postbuildtrigger_param(self, job_type_list, threshold_name):
        """
        Generates config for postbuildtrigger plugin

        :param job_type_list: list with job types (short) to trigger, ``list``
        :param threshold_name: when to trigger projects, ``str``

        :returns: configuration of postbuildtrigger plugin, ``str``
        """

        if job_type_list == []:
            return ''
        elif threshold_name == '':
            raise Exception('No treshold for postbuildtrigger given')
        elif threshold_name not in ['SUCCESS', 'UNSTABLE', 'FAILURE']:  # TODO check tresholds
            raise Exception("Threshold argument invalid")

        postbuildtrigger = self.job_config_params['postbuildtrigger'].replace('@(CHILD_PROJECTS)',
                                                                              self.generate_job_list_string(job_type_list))
        return postbuildtrigger.replace('@(THRESHOLD)', threshold_name)

    def generate_pipelinetrigger_param(self, job_type_list):
        """
        Generates config for pipelinetrigger plugin

        :param job_type_list: list with job types (short) to trigger, ``list``

        :returns: configuration of pipelinetrigger plugin, ``str``
        """

        if job_type_list == []:
            return ''
        return self.job_config_params['pipelinetrigger'].replace('@(PIPELINETRIGGER_PROJECT)',
                                                                 self.generate_job_list_string(job_type_list))

    def generate_groovypostbuild_param(self, script_type, project_list, behavior):
        """
        Generates config for groovypostbuild plugin

        :param script_type: enable, join_enable, disable, ``str``
        :param project_list: list with names of projects, ``list``
        :param behavior: when to execute script (0, 1, 2), ``int``

        :returns: configuration of groovypostbuild plugin, ``str``
        """

        if project_list == []:
            raise Exception('No project is given')
        if behavior > 2 or behavior < 0 or type(behavior) != int:
            raise Exception('Invalid behavior number given')
        script = self.job_config_params['groovypostbuild']['script'][script_type].replace('@(PROJECT_LIST)',
                                                                                          str(self.generate_job_list(project_list)))
        return self.job_config_params['groovypostbuild']['basic'].replace('@(GROOVYPB_SCRIPT)', script).replace('@(GROOVYPB_BEHAVIOR)', str(behavior))

    def generate_parameterizedtrigger_param(self, job_type_list, condition='SUCCESS', predefined_param='', subset_filter='', no_param=False):
        """
        Generates config for parameterizedtrigger plugin

        :param job_type_list: list with job types (short) to trigger, ``list``
        :param condition: when to trigger, ``str``
        :param predefined_param: parameter to pass to downstream project, ``str``
        :param subset_filter: combination filter for matrix projects, ``str``
        :param no_param: trigger build without parameters, ``bool``

        :returns: configuration of parameterizedtrigger plugin, ``str``
        """

        matrix_subset = ''
        if subset_filter != '':
            matrix_subset = self.job_config_params['parameterizedtrigger']['matrix_subset']
            matrix_subset = matrix_subset.replace('@(FILTER)', subset_filter)

        predef_param = ''
        if predefined_param != '':
            predef_param = self.job_config_params['parameterizedtrigger']['predef_param']
            predef_param = predef_param.replace('@(PARAMETER)', predefined_param)

        param_trigger = self.job_config_params['parameterizedtrigger']['basic']
        param_trigger = param_trigger.replace('@(CONFIGS)', predef_param + matrix_subset)
        param_trigger = param_trigger.replace('@(PROJECTLIST)', self.generate_job_list_string(job_type_list))
        param_trigger = param_trigger.replace('@(CONDITION)', condition)

        if no_param:
            param_trigger = param_trigger.replace('@(NOPARAM)', 'true')
        else:
            param_trigger = param_trigger.replace('@(NOPARAM)', 'false')

        return param_trigger


class Pipe_Starter_General_Job(Jenkins_Job):
    """
    Object representation of a general Pipe Starter Job
    """
    def __init__(self, jenkins_instance, pipeline_config, repo_list):
        """
        :param jenkins_instance: object of Jenkins server
        :param pipeline_config: config dict, ``dict``
        :param repo_list: list of names of repository to trigger after change, ``list``
        """

        super(Pipe_Starter_General_Job, self).__init__(jenkins_instance, pipeline_config)

        self.job_type = 'pipe'
        self.job_name = self.generate_job_name(self.job_type, suffix='general')

        self.repo_list = repo_list

    def get_prio_subset_filter(self):
        """
        Gets subset filter for priority build
        """

        subset_filter_input = []
        for repo in self.repo_list:
            for rosdistro in self.pipe_inst.repositories[repo].ros_distro:
                subset_filter_input_entry = {}
                subset_filter_input_entry['repository'] = repo
                subset_filter_input_entry['ros_distro'] = rosdistro
                subset_filter_input_entry['ubuntu_distro'] = self.pipe_inst.repositories[repo].prio_ubuntu_distro
                subset_filter_input_entry['arch'] = self.pipe_inst.repositories[repo].prio_arch
                subset_filter_input.append(subset_filter_input_entry)

        return subset_filter_input

    def get_job_type_params(self):
        """
        Generates pipe starter specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'master'
        self.params['PROJECT'] = 'project'

        # generate groovy postbuild script
        self.params['GROOVY_POSTBUILD'] = self.generate_groovypostbuild_param('disable', ['bringup', 'hilevel', 'release'], 2)

        # generate parameterized trigger
        self.params['PARAMETERIZED_TRIGGER'] = self.generate_parameterizedtrigger_param(['prio'],
                                                                                        subset_filter=self.generate_matrix_filter(self.get_prio_subset_filter()))


class Pipe_Starter_Job(Pipe_Starter_General_Job):
    """
    Object representation of Pipe Starter Job
    """
    def __init__(self, jenkins_instance, pipeline_config, repo_list, poll):
        """
        :param jenkins_instance: object of Jenkins server
        :param pipeline_config: config dict, ``dict``
        :param repo_list: list of names of repository to trigger after change, ``list``
        :param poll: name of repository to monitor for changes, ``str``
        """

        super(Pipe_Starter_Job, self).__init__(jenkins_instance, pipeline_config, repo_list)

        self.job_type = 'pipe'
        self.job_name = self.generate_job_name(self.job_type, suffix=poll)

        self.repo_list = repo_list
        self.poll = repo_list[0]
        if poll != repo_list[0]:
            self.poll = poll

    def get_job_type_params(self):
        """
        Generates pipe starter job specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'master'
        self.params['PROJECT'] = 'project'

        self.params['TRIGGER'] = self.job_config_params['triggers']['vcs']

        if self.poll != self.repo_list[0]:
            git_poll_repo = self.job_config_params['vcs']['git']['repo']
            git_poll_repo = git_poll_repo.replace('@(URI)', self.pipe_inst.repositories[self.repo_list[0]].dependencies[self.poll].url)
            git_poll_repo = git_poll_repo.replace('@(BRANCH)', self.pipe_inst.repositories[self.repo_list[0]].dependencies[self.poll].version)
        else:
            git_poll_repo = self.job_config_params['vcs']['git']['repo']
            git_poll_repo = git_poll_repo.replace('@(URI)', self.pipe_inst.repositories[self.repo_list[0]].url)
            git_poll_repo = git_poll_repo.replace('@(BRANCH)', self.pipe_inst.repositories[self.repo_list[0]].version)

        git_branch = self.job_config_params['vcs']['git']['branch'].replace('@(BRANCH)', '')  # TODO check if thats right
        self.params['VCS'] = (self.job_config_params['vcs']['git']['basic'].replace('@(GIT_REPOS)', git_poll_repo)
                              .replace('@(GIT_BRANCHES)', git_branch))

        # generate groovy postbuild script
        self.params['GROOVY_POSTBUILD'] = self.generate_groovypostbuild_param('disable', ['bringup', 'hilevel', 'release'], 2)

        # generate parameterized trigger
        self.params['PARAMETERIZED_TRIGGER'] = self.generate_parameterizedtrigger_param(['prio'],
                                                                                        subset_filter=self.generate_matrix_filter(self.get_prio_subset_filter()),
                                                                                        predefined_param='POLL=' + self.poll)


class Build_Job(Jenkins_Job):
    """
    Class for build jobs
    """
    def __init__(self, jenkins_instance, pipeline_config):
        """
        Creates a build job instance

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(Build_Job, self).__init__(jenkins_instance, pipeline_config)

        self.job_type = 'build'

    def get_job_type_params(self):
        """
        Generates build job specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'build'  # TODO check labels

        # TODO matrix


class Priority_Build_Job(Build_Job):
    """
    Class for priority build jobs
    """
    def __init__(self, jenkins_instance, pipeline_config):
        """
        Creates a priority build job instance

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(Build_Job, self).__init__(jenkins_instance, pipeline_config)

        self.job_type = 'prio'
        self.job_name = self.generate_job_name(self.job_type)

    def get_job_type_params(self):
        """
        Generates priority build job specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'prio_build'  # TODO check labels

        # TODO groovyscript, pipelinetrigger,
