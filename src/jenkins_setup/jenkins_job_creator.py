#!/usr/bin/env python

import datetime
import socket
import pkg_resources
import yaml
import re
import traceback
from jenkins import JenkinsException


class JenkinsJob(object):
    """
    Jenkins job creation class
    """
    def __init__(self, jenkins_instance, pipeline_instance):
        """
        Sets up Jenkins job object
        """

        self.jenkins_instance = jenkins_instance
        self.pipe_inst = pipeline_instance

        self.job_config_params = pkg_resources.resource_string('jenkins_setup', 'templates/job_config_params.yaml')
        self.job_config_params = yaml.load(self.job_config_params)
        self.job_config = pkg_resources.resource_string('jenkins_setup', 'templates/job_config.xml')

        self.params = {}

        self.job_name = None
        self.job_type = None
        self.poll = None
        self.repo_list = None

    def schedule_job(self):
        """
        Creates new or reconfigure existent job

        @return: return message, ``str``
        """
        if self.jenkins_instance.job_exists(self.job_name):
            try:
                self.jenkins_instance.reconfig_job(self.job_name, self.job_config)
                return "Reconfigured job %s" % self.job_name
            except JenkinsException as ex:
                print traceback.format_exc()
                print ex
                return 'Reconfiguration of %s failed: %s' % (self.job_name, ex)
        else:
            try:
                self.jenkins_instance.create_job(self.job_name, self.job_config)
                return "Created job %s" % self.job_name
            except JenkinsException as ex:
                print traceback.format_exc()
                print ex
                return 'Creation of %s failed: %s' % (self.job_name, ex)

    def create_job(self):
        """
        Sets job specific parameter, sets up the job config and creates job
        on Jenkins instance
        """

        self._set_common_params()

        self._set_job_type_params()

        self._replace_placeholder()
        print self.schedule_job()

        return self.job_name

    def delete_job(self):
        """
        Deletes the job defined by the job name

        @return: return message, ``str``
        """

        if self.jenkins_instance.job_exists(self.job_name):
            try:
                self.jenkins_instance.delete_job(self.job_name)
                print "Deleted job %s" % self.job_name
            except Exception as ex:
                print "Deletion of job %s failed: %s" % (self.job_name, ex)
                return ''
            return self.job_name
        else:
            print "Did not delete job %s, because not job did not exist" % self.job_name
            return ''

    def _set_common_params(self):
        """
        Sets all parameters which have to be defined for every job type
        """

        self.params['USERNAME'] = self.pipe_inst.user_name
        #self.params['JOB_TYPE_NAME'] = self.job_type
        self.params['SCRIPT'] = self.job_type
        self.params['NODE_LABEL'] = self.job_type
        self.params['TIME'] = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d %H:%M')
        self.params['HOSTNAME'] = socket.gethostname()
        self.params['PROJECT'] = 'matrix-project'
        self.params['TRIGGER'] = self.job_config_params['triggers']['none']
        self.params['SHELL'] = ''
        self.params['VCS'] = self.job_config_params['vcs']['none']
        self.params['MATRIX'] = ''
        self.params['PARAMETERIZED_JOB_PARAMETERS'] = ''
        self.params['POSTBUILD_TRIGGER'] = ''
        self.params['JOIN_TRIGGER'] = ''
        self.params['PIPELINE_TRIGGER'] = ''
        self.params['GROOVY_POSTBUILD'] = ''
        self.params['PARAMETERIZED_TRIGGER'] = ''
        self.params['JUNIT_TESTRESULTS'] = ''
        self.params['MAILER'] = ''
        self.params['POSTBUILD_TASK'] = ''
        self.params['WARNINGS_PUBLISHER'] = ''
        self.params['CPPCHECK_PUBLISHER'] = ''
        self._set_authorization_matrix_param(['read', 'workspace'])
        self.params['CONCURRENT_BUILD'] = 'false'
        self.params['CUSTOM_WORKSPACE'] = ''
        self.params['QUIET_PERIOD'] = self.job_config_params['quiet_period'].replace('@(QUIET_PERIOD_DURATION)', '5')
        self.params['BLOCKING_UPSTREAM'] = 'true'
        self.params['BLOCKING_DOWNSTREAM'] = 'false'
        self._set_build_timeout()
        self.params['WS_CLEANUP'] = ''
        self.params['COPY_TO_SLAVE'] = ''
        self.params['COPY_TO_MASTER'] = ''

    ###########################################################################
    # helper methods - parameter generation
    ###########################################################################
    def _replace_placeholder(self):
        """
        Replaces placeholder in template with parameters
        """

        for key, value in self.params.iteritems():
            if "@(%s)" % key not in self.job_config:
                raise KeyError("Parameter %s cannot be replaced, because it is not existent" % key)
            self.job_config = self.job_config.replace("@(%s)" % key, value)
        not_replaced_keys = re.findall(r'@\(([A-Z0-9_]+)\)', self.job_config)
        if not_replaced_keys != []:
            raise KeyError("The keys %s cannot be replaced, because the parameters are missing" % (str(not_replaced_keys)))

    def _generate_job_name(self, job_type, suffix=''):
        '''
        Returns a job_name string generated from a job_type plus suffix

        :param job_type: short job type name, ``str``
        :param suffix: will be added to the end of the name, ``str``
        '''

        if suffix != '':
            return '__'.join([self.pipe_inst.user_name, job_type, suffix])
        else:
            return '__'.join([self.pipe_inst.user_name, job_type])

    def _generate_job_list(self, job_type_list):
        '''
        returns a list of job_names generated from a list of job_types
        '''

        if type(job_type_list) != list:
            raise TypeError("Input type is not type 'list'")
        return [self._generate_job_name(job_type) for job_type in job_type_list]

    def _generate_job_list_string(self, job_type_list):
        '''
        returns a string of comma separated job_names generated from a list of job_types
        '''

        return ', '.join(self._generate_job_list(job_type_list))

    def _generate_matrix_filter(self, config, negation=False):
        """
        Returns a groovy combination filter

        :param config: couples of names and values which have to be met, ``dict``
        :param negation: negates the resulting filter, ``bool``
        :returns: matrix config, ``str``
        """

        filter_ = '%s' % ' || '.join(['(%s)' % ' &amp;&amp; '.join(['%s=="%s"' % (key, value)
                                                                   for key, value in i.iteritems()])
                                     for i in config])
        if negation:
            filter_ = '!(%s)' % filter_

        return filter_

    def _generate_matrix_axis(self, axis_name, value_list):
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
        values = ' '.join([self.job_config_params['matrix']['value'].replace('@(VALUE)', value) for value in sorted(value_list)])
        axis = axis.replace('@(VALUES)', values)

        return axis

    def _set_matrix_param(self, name_value_dict_list, labels=None, filter_=None):
        """
        Returns matrix config for given dictionary containing names and values

        @param name_value_dict_list: matrix parameter config
        @type  name_value_dict_list: list
        @param labels: node labels to run builds on
        @type  labels: list
        @param filter_: combination filter
        @type  filter_: str
        """

        axes = ''
        if name_value_dict_list == []:
            return ''
        for name_value_dict in sorted(name_value_dict_list):
            axes += ' '.join([self._generate_matrix_axis(axis_name, axis_values)
                              for axis_name, axis_values in name_value_dict.iteritems()])
        if axes == '':
            return ''

        matrix = self.job_config_params['matrix']['basic']
        matrix = matrix.replace('@(AXES)', axes)
        if labels:
            matrix = matrix.replace('@(NODE)', '<string>%s</string>' % '</string> <string>'.join(label for label in labels))
        else:
            matrix = matrix.replace('@(NODE)', '<string>%s</string>' % self.job_type)
        #same in short: matrix = matrix.replace('@(NODE)', '<string>%s</string>' % ('</string> <string>'.join(label for label in labels) if labels else self.job_type))
        if filter_:
            matrix += ' ' + self.job_config_params['matrix']['filter'].replace('@(FILTER)', filter_)
        elif filter_ == '':
            matrix += ' ' + self.job_config_params['matrix']['filter'].replace('@(FILTER)', 'repository=="NO_ENTRY"')

        self.params['MATRIX'] = matrix

    def _get_matrix_entries(self, job_type=None):
        """
        Gets all repository, ros_distro, ubuntu_distro and arch entries of
        pipeline configuration

        @return type: list of dicts
        """
        dict_list = []
        repositories = []
        ros_distros = []
        ubuntu_distros = []
        archs = []
        i = 0
        if job_type and job_type != 'prio_build':
            for repo in self.pipe_inst.repositories.keys():
                if job_type in self.pipe_inst.repositories[repo].jobs:
                    i += 1
        for repo_name, repo_data, in self.pipe_inst.repositories.iteritems():
            if i > 5 and job_type and job_type != 'prio_build':
                if job_type not in self.pipe_inst.repositories[repo_name].jobs:
                    continue
            if repo_name not in repositories:
                repositories.append(repo_name)
            for ros_distro in repo_data.ros_distro:
                if ros_distro not in ros_distros:
                    ros_distros.append(ros_distro)
            if repo_data.prio_ubuntu_distro not in ubuntu_distros:
                ubuntu_distros.append(repo_data.prio_ubuntu_distro)
            if repo_data.prio_arch not in archs:
                archs.append(repo_data.prio_arch)
            for ubuntu_distro, repo_archs in repo_data.regular_matrix.iteritems():
                if ubuntu_distro not in ubuntu_distros:
                    ubuntu_distros.append(ubuntu_distro)
                for arch in repo_archs:
                    if arch not in archs:
                        archs.append(arch)

        dict_list.append({'repository': repositories})
        dict_list.append({'ros_distro': ros_distros})
        dict_list.append({'ubuntu_distro': ubuntu_distros})
        dict_list.append({'arch': archs})

        return dict_list

    def _set_jointrigger_param(self, job_type_list, unstable_behavior=False, parameterized_trigger=None):
        """
        Sets config for jointrigger plugin

        @param job_type_list: job types (short) to join
        @type  job_type_list: list
        @param unstable_behavior: execute join project even if upstream projects
        where unstable
        @type  unstable_behavior: bool
        @param parameterized_trigger: parameterized trigger configuration
        @type  parameterized_trigger: str
        """

        if job_type_list == [] and not parameterized_trigger:
            raise Exception("Neither jobs trigger nor parameterized trigger configuration given")
        elif type(unstable_behavior) != bool:
            raise Exception("Behavior argument for unstable job result has to be a boolean")

        jointrigger = self.job_config_params['jointrigger'].replace('@(JOIN_PROJECTS)',
                                                                    self._generate_job_list_string(job_type_list))
        if unstable_behavior:
            jointrigger = jointrigger.replace('@(JOIN_UNSTABLE)', 'true')
        else:
            jointrigger = jointrigger.replace('@(JOIN_UNSTABLE)', 'false')

        if parameterized_trigger is not None:
            if parameterized_trigger == '':
                raise Exception("Parameterized trigger configuration string is empty")
            jointrigger = jointrigger.replace('@(PARAMETERIZED_TRIGGER)', parameterized_trigger)
        else:
            jointrigger = jointrigger.replace('@(PARAMETERIZED_TRIGGER)', '')

        self.params['JOIN_TRIGGER'] = jointrigger

    def _set_postbuildtrigger_param(self, job_type_list, threshold_name):
        """
        Sets config for postbuildtrigger plugin

        :param job_type_list: list with job types (short) to trigger, ``list``
        :param threshold_name: when to trigger projects, ``str``
        """

        if job_type_list == []:
            return ''
        elif threshold_name == '':
            raise Exception('No treshold for postbuildtrigger given')
        elif threshold_name not in ['SUCCESS', 'UNSTABLE', 'FAILURE']:
            raise Exception("Threshold argument invalid")

        postbuildtrigger = self.job_config_params['postbuildtrigger'].replace('@(CHILD_PROJECTS)',
                                                                              self._generate_job_list_string(job_type_list))
        self.params['POSTBUILD_TRIGGER'] = postbuildtrigger.replace('@(THRESHOLD)', threshold_name)

    def _set_pipelinetrigger_param(self, job_type_list):
        """
        Sets config for pipelinetrigger plugin

        :param job_type_list: list with job types (short) to trigger, ``list``
        """

        if job_type_list == []:
            return ''
        self.params['PIPELINE_TRIGGER'] = self.job_config_params['pipelinetrigger'].replace('@(PIPELINETRIGGER_PROJECT)',
                                                                                            self._generate_job_list_string(job_type_list))

    def _set_groovypostbuild_param(self, script_type, project_list, behavior):
        """
        Sets config for groovypostbuild plugin

        :param script_type: enable, join_enable, disable, ``str``
        :param project_list: list with names of projects, ``list``
        :param behavior: when to execute script (0, 1, 2), ``int``
        """

        if project_list == []:
            raise Exception('No project is given')
        if behavior > 2 or behavior < 0 or type(behavior) != int:
            raise Exception('Invalid behavior number given')
        script = self.job_config_params['groovypostbuild']['script'][script_type].replace('@(PROJECT_LIST)',
                                                                                          str(self._generate_job_list(project_list)))
        self.params['GROOVY_POSTBUILD'] = self.job_config_params['groovypostbuild']['basic'].replace('@(GROOVYPB_SCRIPT)', script).replace('@(GROOVYPB_BEHAVIOR)', str(behavior))

    def _get_single_parameterizedtrigger(self, job_type_list, condition='SUCCESS', predefined_param='', subset_filter='', no_param=False):
        """
        Gets config for one parameterized trigger

        :param job_type_list: list with job types (short) to trigger, ``list``
        :param condition: when to trigger, ``str``
        :param predefined_param: parameter to pass to downstream project, ``str``
        :param subset_filter: combination filter for matrix projects, ``str``
        :param no_param: trigger build without parameters, ``bool``
        """

        matrix_subset = ''
        if subset_filter != '':
            matrix_subset = self.job_config_params['parameterizedtrigger']['matrix_subset']
            matrix_subset = matrix_subset.replace('@(FILTER)', subset_filter)

        predef_param = ''
        if predefined_param != '':
            predef_param = self.job_config_params['parameterizedtrigger']['predef_param']
            predef_param = predef_param.replace('@(PARAMETER)', predefined_param)

        param_trigger = self.job_config_params['parameterizedtrigger']['trigger']
        param_trigger = param_trigger.replace('@(CONFIGS)', predef_param + matrix_subset)
        param_trigger = param_trigger.replace('@(PROJECTLIST)', self._generate_job_list_string(job_type_list))
        param_trigger = param_trigger.replace('@(CONDITION)', condition)

        if no_param:
            param_trigger = param_trigger.replace('@(NOPARAM)', 'true')
        else:
            param_trigger = param_trigger.replace('@(NOPARAM)', 'false')

        return param_trigger

    def _get_parameterizedtrigger_param(self, trigger_list):
        """
        Gets config for parameterized trigger plugin

        @param trigger_list: strings of trigger config
        @type  trigger_list: list
        """

        if trigger_list == []:
            raise Exception("No trigger config given")
        param_trigger = self.job_config_params['parameterizedtrigger']['basic']
        param_trigger = param_trigger.replace('@(TRIGGERS)', ' '.join(trigger_list))

        return param_trigger

    def _set_parameterizedtrigger_param(self, trigger_list):
        """
        Sets config for parameterized trigger plugin

        @param trigger_list: strings of trigger config
        @type  trigger_list: list
        """

        self.params['PARAMETERIZED_TRIGGER'] = self._get_parameterizedtrigger_param(trigger_list)

    def _set_mailer_param(self, job_name):
        """
        Sets config for mailer

        @param job_name: name of job to be shown in email subject
        @type  job_name: string
        """

        mailer = self.job_config_params['emailext']
        mailer = mailer.replace('@(EMAIL)', self.pipe_inst.email)
        mailer = mailer.replace('@(JOBNAME)', job_name)
        if self.pipe_inst.committer_email_enabled:
            mailer = mailer.replace('@(EMAIL_TO_COMMITTER)', 'true')
        else:
            mailer = mailer.replace('@(EMAIL_TO_COMMITTER)', 'false')

        self.params['MAILER'] = mailer

    def _set_authorization_matrix_param(self, permission_list):
        """
        Sets config for authorization matrix plugin

        @param permission_list: [read|build|workspace]
        @type  permission_list: list
        """

        authorizations = self.job_config_params['authorizationmatrix']['basic']

        authorization = ''
        for permission in permission_list:
            authorization += self.job_config_params['authorizationmatrix'][permission]
            authorization = authorization.replace('@(USERNAME)', self.pipe_inst.user_name)

        self.params['AUTHORIZATIONMATRIX'] = authorizations.replace('@(PERMISSION)', authorization)

    def _set_junit_testresults_param(self, test_results_dir):
        """
        Sets config for junit test result report
        """

        self.params['JUNIT_TESTRESULTS'] = self.job_config_params['junit_testresults'].replace('@(TEST_RESULTS_DIR)', test_results_dir)

    def _set_vcs_trigger_param(self, vcs_type, url, version = None):
        """
        Sets config for trigger parameter

        @param vcs_type: name of vcs type
        @type  vcs_type: str
        @param url: url of vcs repositoy
        @type  url: str
        @param version: version (branch) of vcs repositoy
        @type  version: str, default=None
        """

        self.params['TRIGGER'] = self.job_config_params['triggers']['vcs']

        vcs_config = self.job_config_params['vcs'][vcs_type]
        vcs_config = vcs_config.replace('@(URI)', url)
        if vcs_type != 'svn':
            vcs_config = vcs_config.replace('@(BRANCH)', version)

        self.params['VCS'] = vcs_config

    def _set_shell_param(self, shell_script):
        """
        Sets config for execute shell parameter

        @param shell_script: shell commands to execute
        @type  shell_script: str
        """

        if shell_script == '':
            raise Exception("No shell script is given")

        shell_config = self.job_config_params['execute_shell']

        shell_config = shell_config.replace('@(COMMAND)', shell_script)

        self.params['SHELL'] = shell_config

    def _get_shell_script(self, script_type=None):
        """
        Gets and sets up execute shell script template
        """

        if script_type:
            shell_temp = pkg_resources.resource_string('jenkins_setup', 'templates/execute_shell_' + script_type + '.yaml')
            shell_temp = yaml.load(shell_temp)
            shell_script = shell_temp[script_type]
        else:
            shell_temp = pkg_resources.resource_string('jenkins_setup', 'templates/execute_shell_' + self.job_type + '.yaml')
            shell_temp = yaml.load(shell_temp)
            shell_script = shell_temp[self.job_type]
        shell_script = shell_script.replace('@(SERVERNAME)', self.pipe_inst.server_name)
        shell_script = shell_script.replace('@(USERNAME)', self.pipe_inst.user_name)
        shell_script = shell_script.replace('@(JOB_TYPE_NAME)', self.job_type)
        shell_script = shell_script.replace('@(PIPELINEREPOSOWNER)', self.pipe_inst.pipeline_repos_owner)
        shell_script = shell_script.replace('@(CONFIG_FOLDER)', self.pipe_inst.config_folder)

        return shell_script

    def _get_prio_subset_filter(self):
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

    def _set_build_timeout(self):
        """
        Set the configuration for the build timeout plugin
        """
        self.params['BUILD_TIMEOUT'] = self.job_config_params['build_timeout']


####################
### starter jobs ###
####################
class PipeStarterManualJob(JenkinsJob):
    """
    Object representation of a manual Pipe Starter Job
    """
    def __init__(self, jenkins_instance, pipeline_config, repo_list):
        """
        :param jenkins_instance: object of Jenkins server
        :param pipeline_config: config dict, ``dict``
        """

        super(PipeStarterManualJob, self).__init__(jenkins_instance, pipeline_config)

        self.job_type = 'pipe_starter_manual'
        self.job_name = self._generate_job_name(self.job_type)
        
        self.repo_list = repo_list

    def _set_job_type_params(self):
        """
        Sets pipe starter manual specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'master'
        self.params['PROJECT'] = 'project'

        # sort repo list alphabetical
        repo_list_sorted = sorted(self.repo_list)

        # set parameterized job parameters
        choice_list = []
        for repo in repo_list_sorted:
            choice_list.append(self.job_config_params['parameters']['string'].replace('@(STRING)', repo))
        choices = ' '.join(choice_list)
        self.params['PARAMETERIZED_JOB_PARAMETERS'] = self.job_config_params['parameters']['choice'].replace('@(CHOICES)', choices)

        # set parameterized trigger
        prio_triggers = []
        prio_triggers.append(self._get_single_parameterizedtrigger(['prio_build'],
                                                                   subset_filter='(repository=="${repository}")',
                                                                   predefined_param='repository=$repository'))
        self._set_parameterizedtrigger_param(prio_triggers)

        # set authorization matrix
        self._set_authorization_matrix_param(['read', 'build', 'workspace'])


class PipeStarterManualAllJob(JenkinsJob):
    """
    Object representation of a manual Pipe Starter Job to trigger all jobs
    """
    def __init__(self, jenkins_instance, pipeline_config, repo_list):
        """
        :param jenkins_instance: object of Jenkins server
        :param pipeline_config: config dict, ``dict``
        :param repo_list: list of names of repository to trigger after change, ``list``
        """

        super(PipeStarterManualAllJob, self).__init__(jenkins_instance, pipeline_config)

        self.job_type = 'pipe_starter_manual'
        self.job_name = self._generate_job_name(self.job_type, suffix='all')

        self.repo_list = repo_list

    def _set_job_type_params(self):
        """
        Sets pipe starter specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'master'
        self.params['PROJECT'] = 'project'

        # set parameterized trigger
        prio_triggers = []
        for repo in self.repo_list:
            prio_triggers.append(self._get_single_parameterizedtrigger(['prio_build'],
                                                                       subset_filter='(repository=="%s")' % repo,
                                                                       predefined_param='repository=%s' % repo))
        self._set_parameterizedtrigger_param(prio_triggers)

        # set authorization matrix
        self._set_authorization_matrix_param(['read', 'build', 'workspace'])


class HardwareBuildTrigger(JenkinsJob):
    """
    Class for hardware build trigger jobs
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        super(HardwareBuildTrigger, self).__init__(jenkins_instance, pipeline_config)

        self.job_type = 'pipe_starter_manual'
        self.job_name = self._generate_job_name(self.job_type, suffix='hardware_build')
        self.repo_list = execute_repo_list

    def _set_job_type_params(self):
        """
        Sets hardware build trigger job specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'master'
        self.params['PROJECT'] = 'project'

        # sort repo list alphabetical
        repo_list_sorted = sorted(self.repo_list)

        # set parameterized job parameters
        choice_list = []
        for repo in repo_list_sorted:
            choice_list.append(self.job_config_params['parameters']['string'].replace('@(STRING)', repo))
        choices = ' '.join(choice_list)
        self.params['PARAMETERIZED_JOB_PARAMETERS'] = self.job_config_params['parameters']['choice'].replace('@(CHOICES)', choices)

        # set parameterized trigger
        prio_triggers = []
        prio_triggers.append(self._get_single_parameterizedtrigger(['hardware_build'],
                                                                   subset_filter='(repository=="${repository}")',
                                                                   predefined_param='repository=$repository'))
        self._set_parameterizedtrigger_param(prio_triggers)

        # set authorization matrix
        self._set_authorization_matrix_param(['read', 'build', 'workspace'])


class HardwareTestTrigger(JenkinsJob):
    """
    Class for hardware test trigger jobs
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        super(HardwareTestTrigger, self).__init__(jenkins_instance, pipeline_config)

        self.job_type = 'pipe_starter_manual'
        self.job_name = self._generate_job_name(self.job_type, suffix='hardware_test')
        self.repo_list = execute_repo_list

    def _set_job_type_params(self):
        """
        Sets hardware test trigger job specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'master'
        self.params['PROJECT'] = 'project'

        # sort repo list alphabetical
        repo_list_sorted = sorted(self.repo_list)

        # set parameterized job parameters
        choice_list = []
        for repo in repo_list_sorted:
            choice_list.append(self.job_config_params['parameters']['string'].replace('@(STRING)', repo))
        choices = ' '.join(choice_list)
        self.params['PARAMETERIZED_JOB_PARAMETERS'] = self.job_config_params['parameters']['choice'].replace('@(CHOICES)', choices)

        # set parameterized trigger
        prio_triggers = []
        prio_triggers.append(self._get_single_parameterizedtrigger(['hardware_test'],
                                                                   subset_filter='(repository=="${repository}")',
                                                                   predefined_param='repository=$repository'))
        self._set_parameterizedtrigger_param(prio_triggers)

        # authorization matrix
        self._set_authorization_matrix_param(['read', 'build', 'workspace'])


class PipeStarterSCMJob(JenkinsJob):
    """
    Object representation of Pipe Starter Job
    """
    def __init__(self, jenkins_instance, pipeline_config, scm_trigger_name, scm_trigger):
        """
        :param jenkins_instance: object of Jenkins server
        :param pipeline_config: config dict, ``dict``
        :param scm_trigger_name: string with scm trigger name
        :param scm_trigger: ``dict`` with scm trigger items
        """

        super(PipeStarterSCMJob, self).__init__(jenkins_instance, pipeline_config)

        self.job_type = 'pipe_starter_scm'
        self.scm_trigger_name = scm_trigger_name
        self.scm_trigger = scm_trigger
        self.poll = scm_trigger['repo']
        self.repo_list = scm_trigger['jobs_to_trigger']

        # substitute "/" in branch namens with "", because jenkins job name is not allowed to contain "/"
        branch = self.scm_trigger['version'].replace("/", "")
        
        # set job name
        self.job_name = self._generate_job_name(self.job_type, suffix=self.scm_trigger['user'] + "__" + self.scm_trigger['repo'] + "__" + branch)

    def _set_job_type_params(self):
        """
        Sets pipe starter job specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'master'
        self.params['PROJECT'] = 'project'

        vcs_type = 'git' # FIXME currently hardcoded to git only
        self._set_vcs_trigger_param( vcs_type, self.scm_trigger['url'], self.scm_trigger['version'])

        # set parameterized triggers
        prio_triggers = []
        for repo in self.repo_list:
            prio_triggers.append(self._get_single_parameterizedtrigger(['prio_build'], 
                                                                       subset_filter='(repository=="%s")' % repo,
                                                                       predefined_param='repository=%s' % repo))
        self._set_parameterizedtrigger_param(prio_triggers)

        # set authorization matrix
        self._set_authorization_matrix_param(['read'])


##################
### build jobs ###
##################
class BuildJob(JenkinsJob):
    """
    Class for build jobs
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        """
        Creates a build job instance

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(BuildJob, self).__init__(jenkins_instance, pipeline_config)

        self.repo_list = execute_repo_list

    def _set_job_type_params(self, matrix_filter=None, matrix_job_type=None):
        """
        Sets build job specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'master'
        self.params['POSTBUILD_TASK'] = self.job_config_params['postbuildtask']

        # cleanup workspace
        self.params['WS_CLEANUP'] = self.job_config_params['ws_cleanup']

        # set copy-to-slave plugin parameters
        copy_to_slave = self.job_config_params['copy_to_slave']
        copy_to_slave = copy_to_slave.replace('@(BASETGZ)', '${ubuntu_distro}__${arch}__${ros_distro}')
        self.params['COPY_TO_SLAVE'] = copy_to_slave
        
        # set copy-to-slave plugin parameters for copying back to master
        copy_to_master = self.job_config_params['copy_to_master']
        copy_to_master = copy_to_master.replace('@(SERVERNAME)', self.pipe_inst.server_name)
        copy_to_master = copy_to_master.replace('@(BASETGZ)', self.pipe_inst.user_name + '__${ubuntu_distro}__${arch}__${ros_distro}__${REPOSITORY}')
        self.params['COPY_TO_MASTER'] = copy_to_master

        # set static code analysis publisher
        self.params['WARNINGS_PUBLISHER'] = self.job_config_params['warningspublisher']
        self.params['CPPCHECK_PUBLISHER'] = self.job_config_params['cppcheckpublisher']

        # set matrix
        if not matrix_filter:
            matrix_filter = self._generate_matrix_filter(self._get_prio_subset_filter())
        matrix_entries_dict_list = self._get_matrix_entries(matrix_job_type)
        self._set_matrix_param(matrix_entries_dict_list, filter_=matrix_filter)


class PriorityBuildJob(BuildJob):
    """
    Class for priority build jobs
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        """
        Creates a priority build job instance

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(PriorityBuildJob, self).__init__(jenkins_instance, pipeline_config, execute_repo_list)

        self.job_type = 'prio_build'
        self.job_name = self._generate_job_name(self.job_type)

    def _set_job_type_params(self):
        """
        Sets priority build job specific job configuration parameters
        """

        super(PriorityBuildJob, self)._set_job_type_params()

        # sort repo list alphabetical
        repo_list_sorted = sorted(self.repo_list)

        # set parameterized job parameters
        choice_list = []
        for repo in repo_list_sorted:
            choice_list.append(self.job_config_params['parameters']['string'].replace('@(STRING)', repo))
        choices = ' '.join(choice_list)
        self.params['PARAMETERIZED_JOB_PARAMETERS'] = self.job_config_params['parameters']['choice'].replace('@(CHOICES)', choices)

        # no concurrent build
        #self.params['CONCURRENT_BUILD'] = 'false'
        
        # quiet period
        self.params['QUIET_PERIOD'] = self.job_config_params['quiet_period'].replace('@(QUIET_PERIOD_DURATION)', '1')

        # email
        self._set_mailer_param('Priority Build')

        # set execute shell
        shell_script = self._get_shell_script()
        self._set_shell_param(shell_script)

        # set pipeline trigger
        self._set_pipelinetrigger_param(['hardware_build_trigger', 'nongraphics_test', 'graphics_test'])

        # set parameterized triggers
        parameterized_triggers = []
        parameterized_triggers.append(self._get_single_parameterizedtrigger(['regular_build'],
                                                                            subset_filter='(repository=="$repository")',
                                                                            predefined_param='repository=$repository'))
        parameterized_triggers.append(self._get_single_parameterizedtrigger(['prio_nongraphics_test'],
                                                                            subset_filter='(repository=="$repository")',
                                                                            predefined_param='repository=$repository'))
        parameterized_triggers.append(self._get_single_parameterizedtrigger(['prio_graphics_test'],
                                                                            subset_filter='(repository=="$repository")',
                                                                            predefined_param='repository=$repository'))
        #parameterized_triggers.append(self._get_single_parameterizedtrigger(['hardware_build'],
        #                                                                    subset_filter='(repository=="$repository")',
        #                                                                    predefined_param='repository=$repository'))
        self._set_parameterizedtrigger_param(parameterized_triggers)


class RegularBuildJob(BuildJob):
    """
    Class for regular build jobs
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        """
        Creates a regular build job instance

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(RegularBuildJob, self).__init__(jenkins_instance, pipeline_config, execute_repo_list)

        self.job_type = 'regular_build'
        self.job_name = self._generate_job_name(self.job_type)

    def _set_job_type_params(self):
        """
        Sets regular build job specific job configuration parameters
        """

        matrix_filter = self._generate_matrix_filter(self._get_regular_subset_filter())

        super(RegularBuildJob, self)._set_job_type_params(matrix_filter=matrix_filter, matrix_job_type='regular_build')

        # set blocking behaviour
        self.params['BLOCKING_UPSTREAM'] = 'true'
        self.params['BLOCKING_DOWNSTREAM'] = 'false'

        # email
        self._set_mailer_param('Regular Build')

        # set execute shell
        shell_script = self._get_shell_script()
        self._set_shell_param(shell_script)

        # set pipeline trigger
        self._set_pipelinetrigger_param(['nongraphics_test', 'graphics_test'])

        # set parameterized triggers
        parameterized_triggers = []
        parameterized_triggers.append(self._get_single_parameterizedtrigger(['regular_nongraphics_test'],
                                                                            subset_filter='(repository=="$repository")',
                                                                            predefined_param='repository=$repository'))
        parameterized_triggers.append(self._get_single_parameterizedtrigger(['regular_graphics_test'],
                                                                            subset_filter='(repository=="$repository")',
                                                                            predefined_param='repository=$repository'))
        self._set_parameterizedtrigger_param(parameterized_triggers)

    def _get_regular_subset_filter(self):
        """
        Gets subset filter for regular build
        """

        subset_filter_input = []
        for repo in self.pipe_inst.repositories.keys():
            for rosdistro in self.pipe_inst.repositories[repo].ros_distro:
                for ubuntu_distro, repo_archs in self.pipe_inst.repositories[repo].regular_matrix.iteritems():
                    for repo_arch in repo_archs:
                        subset_filter_input_entry = {}
                        subset_filter_input_entry['repository'] = repo
                        subset_filter_input_entry['ros_distro'] = rosdistro
                        subset_filter_input_entry['ubuntu_distro'] = ubuntu_distro
                        subset_filter_input_entry['arch'] = repo_arch
                        subset_filter_input.append(subset_filter_input_entry)

        return subset_filter_input


#################
### test jobs ###
#################
class TestJob(JenkinsJob):
    """
    Class for test jobs
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        """
        Creates a test job instance

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        @param repo_list: repositories this job will build
        @type  repo_list: list
        """

        super(TestJob, self).__init__(jenkins_instance, pipeline_config)

        self.repo_list = execute_repo_list

        self.job_type = 'test'
        self.job_name = self._generate_job_name(self.job_type)

    def _set_job_type_params(self, matrix_filter=None, matrix_job_type=None):
        """
        Sets test job specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'master'
        self.params['POSTBUILD_TASK'] = self.job_config_params['postbuildtask']

        # set blocking behaviour
        self.params['BLOCKING_UPSTREAM'] = 'true'
        self.params['BLOCKING_DOWNSTREAM'] = 'false'

        # cleanup workspace
        self.params['WS_CLEANUP'] = self.job_config_params['ws_cleanup']

        # set copy-to-slave plugin parameters
        copy_to_slave = self.job_config_params['copy_to_slave']
        copy_to_slave = copy_to_slave.replace('@(BASETGZ)', self.pipe_inst.user_name + '__${ubuntu_distro}__${arch}__${ros_distro}__${REPOSITORY}')
        self.params['COPY_TO_SLAVE'] = copy_to_slave

        # junit test result location
        self._set_junit_testresults_param('test_results')

        # set matrix
        if not matrix_filter:
            matrix_filter = self._generate_matrix_filter(self._get_test_subset_filter())
        matrix_entries_dict_list = self._get_matrix_entries(matrix_job_type)
        self._set_matrix_param(matrix_entries_dict_list, filter_=matrix_filter)

        # set pipeline trigger
        self._set_pipelinetrigger_param(['release'])

    def _get_test_subset_filter(self):
        """
        Gets the subset filter of the given test job (non/graphics_test)

        @param test_type: test job type to calculate subset filter for
        @type  test_type: string

        @return type: list of dicts of subset filter entries
        """

        subset_filter_input = []
        for repo in self.pipe_inst.repositories.keys():
            if '_'.join(self.job_type.split('_')[1:]) in self.pipe_inst.repositories[repo].jobs:
                if self.job_type.split('_')[0] == 'prio':
                    for rosdistro in self.pipe_inst.repositories[repo].ros_distro:
                        subset_filter_input_entry = {}
                        subset_filter_input_entry['repository'] = repo
                        subset_filter_input_entry['ros_distro'] = rosdistro
                        subset_filter_input_entry['ubuntu_distro'] = self.pipe_inst.repositories[repo].prio_ubuntu_distro
                        subset_filter_input_entry['arch'] = self.pipe_inst.repositories[repo].prio_arch
                        subset_filter_input.append(subset_filter_input_entry)

                elif self.job_type.split('_')[0] == 'regular':
                    for rosdistro in self.pipe_inst.repositories[repo].ros_distro:
                        for ubuntu_distro, repo_archs in self.pipe_inst.repositories[repo].regular_matrix.iteritems():
                            for repo_arch in repo_archs:
                                subset_filter_input_entry = {}
                                subset_filter_input_entry['repository'] = repo
                                subset_filter_input_entry['ros_distro'] = rosdistro
                                subset_filter_input_entry['ubuntu_distro'] = ubuntu_distro
                                subset_filter_input_entry['arch'] = repo_arch
                                subset_filter_input.append(subset_filter_input_entry)

                else:
                    raise Exception("Unknown build job type: %s" % self.job_type.split('_')[0])

        return subset_filter_input


#############################
### nongraphics test jobs ###
#############################
class PriorityNongraphicsTestJob(TestJob):
    """
    Class for priority nongraphics test job
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        """
        Creates a nongraphics test job instance for the priority builds

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(PriorityNongraphicsTestJob, self).__init__(jenkins_instance, pipeline_config, execute_repo_list)

        self.job_type = 'prio_nongraphics_test'
        self.job_name = self._generate_job_name(self.job_type)

    def _set_job_type_params(self):
        """
        Sets nongraphics test job specific job configuration parameters
        """

        super(PriorityNongraphicsTestJob, self)._set_job_type_params(matrix_job_type='nongraphics_test')

        # sort repo list alphabetical
        repo_list_sorted = sorted(self.repo_list)

        # set parameterized job parameters
        choice_list = []
        for repo in repo_list_sorted:
            choice_list.append(self.job_config_params['parameters']['string'].replace('@(STRING)', repo))
        choices = ' '.join(choice_list)
        self.params['PARAMETERIZED_JOB_PARAMETERS'] = self.job_config_params['parameters']['choice'].replace('@(CHOICES)', choices)

        # email
        self._set_mailer_param('Priority Non-Graphics Test')

        # set execute shell
        shell_script = self._get_shell_script('nongraphics_test')
        self._set_shell_param(shell_script)


class RegularNongraphicsTestJob(TestJob):
    """
    Class for regular nongraphics test job
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        """
        Creates a nongraphics test job instance for the regular builds

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(RegularNongraphicsTestJob, self).__init__(jenkins_instance, pipeline_config, execute_repo_list)

        self.job_type = 'regular_nongraphics_test'
        self.job_name = self._generate_job_name(self.job_type)

    def _set_job_type_params(self):
        """
        Sets nongraphics test job specific job configuration parameters
        """

        super(RegularNongraphicsTestJob, self)._set_job_type_params(matrix_job_type='nongraphics_test')

        # email
        self._set_mailer_param('Regular Non-Graphics Test')

        # set execute shell
        shell_script = self._get_shell_script('nongraphics_test')
        self._set_shell_param(shell_script)


##########################
### graphics test jobs ###
##########################
class PriorityGraphicsTestJob(TestJob):
    """
    Class for priority graphics test job
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        """
        Creates a graphics test job instance for the priority builds

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(PriorityGraphicsTestJob, self).__init__(jenkins_instance, pipeline_config, execute_repo_list)

        self.job_type = 'prio_graphics_test'
        self.job_name = self._generate_job_name(self.job_type)

    def _set_job_type_params(self):
        """
        Sets graphics test job specific job configuration parameters
        """

        super(PriorityGraphicsTestJob, self)._set_job_type_params(matrix_job_type='graphics_test')

        # sort repo list alphabetical
        repo_list_sorted = sorted(self.repo_list)

        # set parameterized job parameters
        choice_list = []
        for repo in repo_list_sorted:
            choice_list.append(self.job_config_params['parameters']['string'].replace('@(STRING)', repo))
        choices = ' '.join(choice_list)
        self.params['PARAMETERIZED_JOB_PARAMETERS'] = self.job_config_params['parameters']['choice'].replace('@(CHOICES)', choices)

        # email
        self._set_mailer_param('Priority Graphics Test')

        # set execute shell
        shell_script = self._get_shell_script('graphics_test')
        self._set_shell_param(shell_script)

        # set pipeline trigger
        self._set_pipelinetrigger_param(['release'])


class RegularGraphicsTestJob(TestJob):
    """
    Class for regular graphics test job
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        """
        Creates a graphics test job instance for the regular builds

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(RegularGraphicsTestJob, self).__init__(jenkins_instance, pipeline_config, execute_repo_list)

        self.job_type = 'regular_graphics_test'
        self.job_name = self._generate_job_name(self.job_type)

    def _set_job_type_params(self):
        """
        Sets graphics test job specific job configuration parameters
        """

        super(RegularGraphicsTestJob, self)._set_job_type_params(matrix_job_type='graphics_test')

        # email
        self._set_mailer_param('Regular Graphics Test')

        # set execute shell
        shell_script = self._get_shell_script('graphics_test')
        self._set_shell_param(shell_script)


#####################
### hardware jobs ###
#####################
class HardwareJob(JenkinsJob):
    """
    Class for hardware jobs
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        """
        Creates a hardware job instance

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        @param execute_repo_list: List of repositories
        @type  execute_repo_list: list of strings
        """

        super(HardwareJob, self).__init__(jenkins_instance, pipeline_config)

        self.repo_list = execute_repo_list

    def _get_hardware_matrix_entries(self):
        """
        Gets all repository to build and all robots to build on

        @return type: list of dicts
        """

        dict_list = []
        repositories = []
        ros_distros = []
        robots = []
        for repo in self.pipe_inst.repositories.keys():
            if 'hardware_build' in self.pipe_inst.repositories[repo].jobs:
                repositories.append(repo)
                robot = self.pipe_inst.repositories[repo].robots
                #for robot in self.pipe_inst.repositories[repo].robots:
                #    if robot not in robots:
                #        robots.append(robot)
                if robot not in robots:
                    robots.append(robot)

                ros_distro = self.pipe_inst.repositories[repo].ros_distro
                for rd in ros_distro:
                    if rd not in ros_distros:
                        ros_distros.append(rd)

        dict_list.append({'repository': repositories})
        dict_list.append({'ros_distro': ros_distros})

        return (dict_list, robots)

    def _get_hardware_subset_filter(self):
        """
        Gets subset filter for hardware jobs
        """

        subset_filter_input = []
        for repo in self.pipe_inst.repositories.keys():
            for rosdistro in self.pipe_inst.repositories[repo].ros_distro:
                if 'hardware_build' in self.pipe_inst.repositories[repo].jobs:
                    subset_filter_input_entry = {}
                    subset_filter_input_entry['repository'] = repo
                    subset_filter_input_entry['ros_distro'] = rosdistro
                    subset_filter_input_entry['label'] = self.pipe_inst.repositories[repo].robots#[0]  # FIXME hack as long as robots attribute is list not str
                    subset_filter_input.append(subset_filter_input_entry)

        return subset_filter_input


class HardwareBuildJob(HardwareJob):
    """
    Class for hardware build jobs
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        """
        Creates a hardware build job

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(HardwareBuildJob, self).__init__(jenkins_instance, pipeline_config, execute_repo_list)

        self.job_type = 'hardware_build'
        self.job_name = self._generate_job_name(self.job_type)

    def _set_job_type_params(self):
        """
        Sets hardware build job specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'master'

        # cleanup workspace
        #self.params['WS_CLEANUP'] = self.job_config_params['ws_cleanup']

        # set copy-to-slave plugin parameters
        copy_to_slave = self.job_config_params['copy_to_slave_hwjobs']
        self.params['COPY_TO_SLAVE'] = copy_to_slave

        # set blocking behaviour
        self.params['BLOCKING_UPSTREAM'] = 'false'
        self.params['BLOCKING_DOWNSTREAM'] = 'true'

        # set matrix
        matrix_filter = self._generate_matrix_filter(self._get_hardware_subset_filter())
        (matrix_entries_dict_list, robots) = self._get_hardware_matrix_entries()
        self._set_matrix_param(matrix_entries_dict_list, labels=robots, filter_=matrix_filter)

        # set custom workspace
        self.params['CUSTOM_WORKSPACE'] = self.job_config_params['custom_workspace'].replace('@(CUSTOM_WORKSPACE_DIR)', 'workspace_hardware_build').replace('@(CUSTOM_WORKSPACE_CHILD_DIR)', self.pipe_inst.user_name)

        # email
        self._set_mailer_param('Hardware Build')

        # set execute shell
        shell_script = self._get_shell_script()
        self._set_shell_param(shell_script)

        # authorization matrix
        self._set_authorization_matrix_param(['read', 'workspace'])


class HardwareTestJob(HardwareJob):
    """
    Class for hardware test jobs
    """
    def __init__(self, jenkins_instance, pipeline_config, execute_repo_list):
        """
        Creates a hardware test job

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(HardwareTestJob, self).__init__(jenkins_instance, pipeline_config, execute_repo_list)

        self.job_type = 'hardware_test'
        self.job_name = self._generate_job_name(self.job_type)

    def _set_job_type_params(self):
        """
        Sets hardware job specific job configuration parameters
        """

        self.params['NODE_LABEL'] = 'master'

        # set copy-to-slave plugin parameters
        copy_to_slave = self.job_config_params['copy_to_slave_hwjobs']
        self.params['COPY_TO_SLAVE'] = copy_to_slave
        
        # set blocking behaviour
        self.params['BLOCKING_UPSTREAM'] = 'true'
        self.params['BLOCKING_DOWNSTREAM'] = 'false'

        # set matrix
        matrix_filter = self._generate_matrix_filter(self._get_hardware_subset_filter())
        (matrix_entries_dict_list, robots) = self._get_hardware_matrix_entries()
        self._set_matrix_param(matrix_entries_dict_list, labels=robots, filter_=matrix_filter)

        # junit test result location
        self._set_junit_testresults_param('${repository}/current/test_results')

        # set custom workspace
        self.params['CUSTOM_WORKSPACE'] = self.job_config_params['custom_workspace'].replace('@(CUSTOM_WORKSPACE_DIR)', 'workspace_hardware_build').replace('@(CUSTOM_WORKSPACE_CHILD_DIR)', self.pipe_inst.user_name)

        # email
        self._set_mailer_param('Hardware Test')

        # set execute shell
        shell_script = self._get_shell_script()
        self._set_shell_param(shell_script)

        # set pipeline trigger
        self._set_pipelinetrigger_param(['release'])

        # authorization matrix
        self._set_authorization_matrix_param(['read', 'workspace'])

#######################
### deployment jobs ###
#######################
class DeploymentJob(JenkinsJob):
    """
    """
    def __init__(self, jenkins_instance, pipeline_config):
        super(DeploymentJob, self).__init__(jenkins_instance, pipeline_config)

        self.job_type = 'deployment'
        self.job_name = self._generate_job_name(self.job_type)

    def _set_job_type_params(self):
        """
        Sets deployment job specific job configuration parameters
        """

        #self.params['NODE_LABEL'] = 'master'
        self.params['PROJECT'] = 'project' #TODO what do we need this for???
        
        # set copy-to-slave plugin parameters
        copy_to_slave = self.job_config_params['copy_to_slave']
        copy_to_slave = copy_to_slave.replace('@(SERVERNAME)', 'fmw-xps') # FIXME: replace servername with real data
        copy_to_slave = copy_to_slave.replace('@(BASETGZ)', 'ipa-fmw__precise__amd64__hydro__fiad_scenario') # FIXME: replace basetgz with real data
        self.params['COPY_TO_SLAVE'] = copy_to_slave

        # set execute shell
        shell_script = self._get_shell_script()
        self._set_shell_param(shell_script)

        # authorization matrix
        self._set_authorization_matrix_param(['read', 'build', 'workspace'])


class ReleaseJob(JenkinsJob):
    """
    Class for release jobs
    """
    def __init__(self, jenkins_instance, pipeline_config):
        """
        Creates a release job

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(ReleaseJob, self).__init__(jenkins_instance, pipeline_config)

        self.job_type = 'release'
        self.job_name = self._generate_job_name(self.job_type)

    def _set_job_type_params(self):
        """
        Sets release job specific job configuration parameters
        """

        self.params['PROJECT'] = 'project'
        self.params['NODE_LABEL'] = 'release'

        # email
        self._set_mailer_param('Release')


class CleanUpJob(JenkinsJob):
    """
    Class for clean up jobs
    """
    def __init__(self, jenkins_instance, pipeline_config):
        """
        Creates a clean up job

        @param jenkins_instance: Jenkins instance
        @type  jenkins_instance: jenkins.Jenkins
        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        super(CleanUpJob, self).__init__(jenkins_instance, pipeline_config)

    def _set_job_type_params(self):
        """
        Sets clean up job specific job configuration parameters
        """

        self.params['PROJECT'] = 'project'
        self.params['NODE_LABEL'] = 'clean_up'

# TODO classes: release
