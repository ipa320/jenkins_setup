#!/usr/bin/env python

import socket
import yaml
import datetime
#import urllib
import re
import pkg_resources

from jenkins_setup import cob_distro


class Jenkins_Jobs(object):

    def __init__(self, jenkins_instance, buildpipeline_config):
        # parse target yaml files from github/ros/rosdistro TODO disabled for
        # testing; later used with a 'try' statement
        #try:
        #    f = urllib.urlopen('https://raw.github.com/ros/rosdistro/master/releases/targets.yaml')
        #    for i in yaml.load(f.read()):
        #        self.TARGET_PLATFORM = dict(self.TARGET_PLATFORM.items() + i.items())
        #except:
        self.TARGET_PLATFORM = {'electric': ['lucid', 'natty', 'oneiric'],
                                'fuerte': ['lucid', 'oneiric', 'precise'],
                                'groovy': ['oneiric', 'precise', 'quantal']}
        self.TARGET_ARCH = ['i386', 'amd64']

        self.PRIO_DISTRO = {'electric': 'natty',
                            'fuerte': 'oneiric',
                            'groovy': 'oneiric'}
        self.PRIO_ARCH = 'amd64'

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
        self.JOB_TYPE_PARAMS = {'pipe_starter': self.pipe_starter_params,
                                'prio_build': self.prio_build_params,
                                'normal_build': self.normal_build_params,
                                #'downstream_build': self.downstream_build_params,
                                #'database_test': self.database_test_params,
                                #'simulation_test': self.simulation_test_params,
                                #'application_test': self.application_test_params,
                                #'cleanup': self.cleanup_params,
                                #'bringup_hardward_test': self.bringup_hardware_test_params,
                                #'highlevel_hardward_test': self.highlevel_hardware_test_params,
                                #'release': self.release_params
                                }

        self.jenkins_instance = jenkins_instance
        self.pipe_conf = buildpipeline_config
        self.pipe_instance = cob_distro.Cob_Distro_Pipe()
        self.pipe_instance.load_from_dict(self.pipe_conf['repositoties'])

        job_config_params = pkg_resources.resource_string('jenkins_setup', 'templates/job_config_params.yaml')
        self.jcp = yaml.load(job_config_params)

    ###########################################################################
    # job creation main method
    ###########################################################################
    def create_job(self, job_type):  # TODO
        '''
        Create new job
        '''

        job_config = pkg_resources.resource_string('jenkins_setup', 'templates/job_config.xml')

        params = self.get_common_params(job_type)

        params = self.get_job_type_params(job_type, params)

        job_config = self.replace_placeholder(job_config, params)

        job_name = self.generate_job_name(self.pipe_conf, job_type)

        self.schedule_job(job_name, job_config)

    def schedule_job(self, job_name, job_config):
        '''TODO'''
        if self.jenkins_instance.job_exists(job_name):
            try:
                self.jenkins_instance.reconfig_job(job_name, job_config)
                print "Reconfigured job %s" % job_name
                return 'reconfigured'
            except:
                return 'reconfiguration failed'
        else:
            try:
                self.jenkins_instance.create_job(job_name, job_config)
                print "Created job %s" % job_name
                return 'created'
            except:
                return 'creation failed'

    def delete_job(self, job_name):
        '''TODO'''
        if self.jenkins_instance.job_exists(job_name):
            try:
                self.jenkins_instance.delete_job(job_name)
            except:
                return 'deletion failed'  # Exception ??
            return 'deleted'
        else:
            return 'not existent'

    ###########################################################################
    # configure job
    ###########################################################################
    def get_common_params(self, job_type):
        '''TODO'''
        params = {}
        params['USERNAME'] = self.pipe_conf['user_name']
        params['EMAIL'] = self.pipe_conf['email']
        params['EMAIL_COMMITTER'] = self.pipe_conf['committer']
        params['JOB_TYPE_NAME'] = self.JOB_TYPE_NAMES[job_type]
        params['SCRIPT'] = self.JOB_TYPE_NAMES[job_type]
        params['NODE_LABEL'] = job_type
        params['TIME'] = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d %H:%M')
        params['HOSTNAME'] = socket.gethostname()
        params['ROSDISTRO'] = self.pipe_conf['ros_distro']
        params['PROJECT'] = 'matrix-project'
        params['TRIGGER'] = self.jcp['triggers']['none']
        params['VCS'] = self.jcp['vcs']['none']
        params['MATRIX'] = ''
        params['PARAMETERS'] = ''
        params['POSTBUILD_TRIGGER'] = params['JOIN_TRIGGER'] = params['PIPELINE_TRIGGER'] = params['GROOVY_POSTBUILD'] = ''

        return params

    def get_job_type_params(self, job_type, params):
        '''TODO'''
        return self.JOB_TYPE_PARAMS[self.JOB_TYPE_NAMES[job_type]](params)

    def pipe_starter_params(self, params):
        '''
        Generates a pipe starter job configuration by setting the specific
        parameters defined in pipeline_config.yaml

        :param params: dictionary with all available configuration parameters,
        ``dict``
        :returns: set up configuration parameters, ``dict``
        '''

        params['NODE_LABEL'] = 'master'
        params['PROJECT'] = 'project'

        params['TRIGGER'] = self.jcp['triggers']['vcs']

        # maybe the repo url itself has to generated TODO
        git_repo_list = [self.jcp['vcs']['git']['repo'].replace('@(URI)', self.pipe_conf['repositories'][repo]['url'])
                         .replace('@(BRANCH)', self.pipe_conf['repositories'][repo]['version'])
                         for repo in self.pipe_conf['repositories']]
        git_repos = self.jcp['vcs']['git']['repos'].replace('@(GIT_REPO)', ' '.join(git_repo_list))
        git_branches = self.jcp['vcs']['git']['branch'].replace('@(BRANCH)', '')  # TODO check if thats right
        params['VCS'] = (self.jcp['vcs']['git']['basic'].replace('@(GIT_REPOS)', git_repos)
                         .replace('@(GIT_BRANCHES)', git_branches))

        # generate groovy postbuild script
        params['GROOVY_POSTBUILD'] = self.generate_groovypostbuild_param('disable', ['bringup', 'hilevel', 'release'], 2)

        # generate postbuild trigger
        params['POSTBUILD_TRIGGER'] = self.generate_postbuildtrigger_param(['prio'], 'SUCCESS')

        return params

    def prio_build_params(self, params):
        ''' TODO '''

        params['NODE_LABEL'] = 'prio_build'  # necessary?

        filter = self.generate_matrix_filter(self.generate_prio_distro_arch())
        params['MATRIX'] = self.generate_matrix_param(self.generate_distro_arch_field(), filter)

        #replace script_args in command TODO
        params['SCRIPT'] = 'prio_build_script'
        #params['SCRIPT_ARGS'] = bpc['pipeline_github_url'] $ROSDISTRO $REPOLIST
        #params['COMMAND'] = shell script to start run_chroot_local PRIO_UBUNTU_DISTRO PRIO_ARCH
        #                               $WORKSPACE params['SCRIPT']
        #                               params['SCRIPT_ARGS'] TODO

        # generate groovy postbuild script
        params['GROOVY_POSTBUILD'] = self.generate_groovypostbuild_param('enable', ['bringup'], 2)

        # generate postbuild trigger
        params['POSTBUILD_TRIGGER'] = self.generate_postbuildtrigger_param(['down'], 'SUCCESS')

        # generate pipeline trigger
        params['PIPELINE_TRIGGER'] = self.generate_pipelinetrigger_param(['bringup'])

        return params

    def normal_build_params(self, params):
        ''' TODO '''

        params['NODE_LABEL'] = 'build'

        # generate matrix
        filter = self.generate_matrix_filter(self.generate_prio_distro_arch(), True)
        params['MATRIX'] = self.generate_matrix_param(self.generate_distro_arch_field(), filter)

        return params

    def downstream_buils_params(self, params):
        ''' TODO '''

        params['NODE_LABEL'] = 'build'

        # generate postbuild trigger
        params['POSTBUILD_TRIGGER'] = self.generate_postbuildtrigger_param(['db', 'sim'], 'SUCCESS')

        # generate join trigger
        params['JOIN_TRIGGER'] = self.generate_jointrigger_param(['app'], 'true')

        return params

    ###########################################################################
    # helper methods - parameter generation
    ###########################################################################
    def replace_placeholder(self, job_config, params):
        '''replace placeholder in template with params'''
        for key, value in params.iteritems():
            if "@(%s)" % key not in job_config:
                raise KeyError("Parameter %s could not be replaced, because it is not existent" % key)
            job_config = job_config.replace("@(%s)" % key, value)
        not_replaced_keys = re.findall('@\(([A-Z0-9_]+)\)', job_config)
        if not_replaced_keys != []:
            raise KeyError("The keys %s were not replaced, because the parameters where missing" % (str(not_replaced_keys)))
        return job_config

    def generate_job_name(self, job_type):
        '''returns a job_name string generated from a job_type'''
        return '__'.join([self.pipe_conf['user_name'],
                          self.JOB_TYPE_NAMES[job_type]])

    def generate_job_list(self, job_type_list):
        '''returns a list of job_names generated from a list of job_types'''
        if type(job_type_list) != list:
            raise TypeError("Input type is not type 'list'")
        return [self.generate_job_name(job_type) for job_type in job_type_list]

    def generate_job_list_string(self, job_type_list):
        '''returns a string of comma separated job_names generated from a list of job_types'''
        return ', '.join(self.generate_job_list(job_type_list))

    def generate_distro_arch_field(self):
        '''TODO'''
        result = {}
        result['ros_distro'] = self.pipe_conf['ros_distro']
        result['ubuntu_distro'] = []
        for ros_distro in result['ros_distro']:
            for ubuntu_distro in self.TARGET_PLATFORM[ros_distro]:
                if ubuntu_distro not in result['ubuntu_distro']:
                    result['ubuntu_distro'].append(ubuntu_distro)
        result['arch'] = self.TARGET_ARCH
        return result

    def generate_prio_distro_arch(self):
        '''TODO'''
        result = []
        for ros_distro in self.pipe_conf['ros_distro']:
            entry = {}
            entry['ros_distro'] = ros_distro
            entry['ubuntu_distro'] = self.PRIO_DISTRO[ros_distro]
            entry['arch'] = self.PRIO_ARCH
            if ros_distro in self.pipe_conf['prio_distro_arch']:
                entry['ubuntu_distro'] = self.pipe_conf['prio_distro_arch'][ros_distro]['ubuntu_distro']
                entry['arch'] = self.pipe_conf['prio_distro_arch'][ros_distro]['arch']

            result.append(entry)

        return result

    def generate_matrix_filter(self, config, negation=False):
        '''TODO'''
        if negation:
            filter = '!(%s)' % ' || '.join(['(%s)' % ' && '.join(['%s == %s' % (key, value)
                                                                  for key, value in i.iteritems()])
                                            for i in config])
        else:
            filter = '(%s)' % ' || '.join(['(%s)' % ' && '.join(['%s == %s' % (key, value)
                                                                 for key, value in i.iteritems()])
                                           for i in config])

        return filter

    def generate_matrix_axis(self, axis_name, value_list):
        ''' TODO '''

        if axis_name == '':
            raise Exception('No proper name given')
        if value_list == []:
            raise Exception('No values given')
        axis = self.jcp['matrix']['axis'].replace('@(NAME)', axis_name)
        values = ' '.join([self.jcp['matrix']['value'].replace('@(VALUE)', value) for value in value_list])
        axis = axis.replace('@(VALUES)', values)

        return axis

    def generate_matrix_param(self, name_value_dict, filter=None):
        ''' TODO '''

        axes = ''
        if name_value_dict == {}:
            return ''
        axes += ' '.join([self.generate_matrix_axis(axis_name, axis_values)
                          for axis_name, axis_values in name_value_dict.iteritems()])

        matrix = self.jcp['matrix']['basic'].replace('@(AXES)', axes)
        if filter:
            matrix += ' ' + self.jcp['matrix']['filter'].replace('@(FILTER)', filter)

        return matrix

    def generate_jointrigger_param(self, job_type_list, unstable_behavior):
        ''' TODO '''
        if job_type_list == []:
            return ''
        elif unstable_behavior == '':
            raise Exception('No behavior for unstable job result')
        elif not (unstable_behavior == 'true' or unstable_behavior == 'false'):
            raise Exception("Behavior argument has to be 'true' or 'false'")

        jointrigger = self.jcp['jointrigger'].replace('@(JOIN_PROJECTS)',
                                                      self.generate_job_list_string(job_type_list))
        return jointrigger.replace('@(JOIN_UNSTABLE)', str(unstable_behavior))

    def generate_postbuildtrigger_param(self, job_type_list, threshold_name):
        ''' TODO '''
        if job_type_list == []:
            return ''
        elif threshold_name == '':
            raise Exception('No treshold for postbuildtrigger given')
        elif threshold_name not in ['SUCCESS', 'UNSTABLE', 'FAILURE']:  # TODO check tresholds
            raise Exception("Threshold argument invalid")

        postbuildtrigger = self.jcp['postbuildtrigger'].replace('@(CHILD_PROJECTS)',
                                                                self.generate_job_list_string(job_type_list))
        return postbuildtrigger.replace('@(THRESHOLD)', threshold_name)

    def generate_pipelinetrigger_param(self, job_type_list):
        ''' TODO '''
        if job_type_list == []:
            return ''
        return self.jcp['pipelinetrigger'].replace('@(PIPELINETRIGGER_PROJECT)',
                                                   self.generate_job_list_string(job_type_list))

    def generate_groovypostbuild_param(self, script_type, project_list, behavior):
        ''' TODO '''
        if project_list == []:
            raise Exception('No project is given')
        if behavior > 2 or behavior < 0 or type(behavior) != int:
            raise Exception('Invalid behavior number given')
        script = self.jcp['groovypostbuild']['script'][script_type].replace('@(PROJECT_LIST)',
                                                                            str(self.generate_job_list(project_list)))
        return self.jcp['groovypostbuild']['basic'].replace('@(GROOVYPB_SCRIPT)', script).replace('@(GROOVYPB_BEHAVIOR)', str(behavior))
