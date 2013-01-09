#!/usr/bin/env python

import unittest

import os
import yaml
import datetime
import socket
import jenkins

from jenkins_setup import run_jenkins_job_creation


class Job_Generation_Test(unittest.TestCase):

    def setUp(self):
        self.maxDiff = None
        with open('pipeline_config.yaml') as f:
            self.test_dict = yaml.load(f)
        self.job_type_test_list = ['pipe', 'prio', 'normal']

        with open(os.path.expanduser('~/jenkins-config/slave_config.yaml')) as f:
            info = yaml.load(f)
        jenkins_instance = jenkins.Jenkins(info['master_url'], info['jenkins_login'], info['jenkins_pw'])
        self.jj = run_jenkins_job_creation.Jenkins_Jobs(jenkins_instance, self.test_dict)

        self.jj.PRIO_DISTRO['test_rosdistro'] = 'natty'
        self.jj.TARGET_PLATFORM['test_rosdistro'] = ['natty', 'oneiric', 'precise']

    # Testing create_job
    #def test__create_job__input_job_type__job_created(self):

    #def test__create_job__input_job_type__job_reconfigured(self)

    # Testing generate_job_name
    def test__generate_job_name__input_job_string__return_job_name_string(self):
        result = self.jj.generate_job_name('pipe')
        self.assertEqual(result, 'test_user__pipe_starter')

    def test__generate_job_name__input_job_string__return_job_name_string2(self):
        result = self.jj.generate_job_name('hilevel')
        self.assertEqual(result, 'test_user__highlevel_hardware_test')

    def test__generate_job_name__input_wrong_key_string__raise_exception(self):
        self.assertRaises(KeyError, self.jj.generate_job_name, 'wrong_key')

    def test__generate_job_name__input_wrong_key_int__raise_exception(self):
        self.assertRaises(KeyError, self.jj.generate_job_name, 12)

    # Testing generate_job_list
    def test__generate_job_list__input_job_list__return_job_name_list(self):
        result = self.jj.generate_job_list(self.job_type_test_list)
        self.assertEqual(result, ['test_user__pipe_starter',
                                  'test_user__prio_build',
                                  'test_user__normal_build'])

    def test__generate_job_list__input_empty_list__return_empty_string(self):
        result = self.jj.generate_job_list([])
        self.assertEqual(result, [])

    def test__generate_job_list__input_dict__raise_exception(self):
        self.assertRaises(TypeError, self.jj.generate_job_list, {'pipe', 'prio', 'normal'})

    # Testing generate_job_list_string
    def test__generate_job_list_string__input_job_list__return_job_names_as_comma_separatet_string(self):
        result = self.jj.generate_job_list_string(self.job_type_test_list)
        self.assertEqual(result, 'test_user__pipe_starter, test_user__prio_build, test_user__normal_build')

    def test__generate_job_list_string__input_empty_list__return_empty_string(self):
        result = self.jj.generate_job_list_string([])
        self.assertEqual(result, '')

    # Testing replace_placeholder
    def test__replace_placeholder__input_config_string_and_params_dict__resturn_config_string(self):
        to_replace = '@(TEST_1) and @(TEST_2) should be replaced'
        result = self.jj.replace_placeholder(to_replace, {'TEST_1': 'THIS', 'TEST_2': 'THAT'})
        self.assertEqual(result, 'THIS and THAT should be replaced')

    def test__replace_placeholder__input_not_existent_key__raise_exception(self):
        to_replace = '@(TEST_1) and @(TEST_2) should be replaced'
        self.assertRaises(KeyError, self.jj.replace_placeholder, to_replace, {'WRONG': 'THIS', 'TEST_2': 'THAT'})

    def test__replace_placeholder__missing_parameter__raise_exception(self):
        to_replace = '@(TEST_1) and @(TEST_2) should be replaced'
        self.assertRaises(KeyError, self.jj.replace_placeholder, to_replace, {'TEST_1': 'THIS'})

    def test__replace_placeholder__missing_parameter__raise_exception2(self):
        to_replace = '@(TEST_1) and @(TEST_2) should be @( replaced'
        result = self.jj.replace_placeholder(to_replace, {'TEST_1': 'THIS', 'TEST_2': 'THAT'})
        self.assertEqual(result, 'THIS and THAT should be @( replaced')

    # Testing generate_matrix_axis
    def test__generate_matrix_axis__input_name_string_and_value_list__return_axis_config_string(self):
        axis_name = 'test_axis'
        value_list = ['test_value_1', 'test_value_2', 'test_value_3']
        result = self.jj.generate_matrix_axis(axis_name, value_list)
        self.assertEqual(result, '<hudson.matrix.TextAxis> <name>test_axis</name> <values> <string>test_value_1</string> <string>test_value_2</string> <string>test_value_3</string> </values> </hudson.matrix.TextAxis>')

    def test__generate_matrix_axis__input_empty_list__raise_exception(self):
        axis_name = 'test_axis'
        self.assertRaises(Exception, self.jj.generate_matrix_axis, axis_name, [])

    def test__generate_matrix_axis__input_empty_name__raise_exception(self):
        value_list = ['test_value_1', 'test_value_2', 'test_value_3']
        self.assertRaises(Exception, self.jj.generate_matrix_axis, '', value_list)

    # Testing generate_matrix_param
    def test__generate_matrix_param__input_name_value_dict_and_filter_string__return_matrix_config_string(self):
        name_value_test_dict = {'test_axis': ['test_value_1', 'test_value_2', 'test_value_3']}
        result = self.jj.generate_matrix_param(name_value_test_dict, 'filter')
        self.assertEqual(result, '<axes> <hudson.matrix.TextAxis> <name>test_axis</name> <values> <string>test_value_1</string> <string>test_value_2</string> <string>test_value_3</string> </values> </hudson.matrix.TextAxis> </axes> <combinationFilter>filter</combinationFilter>')

    def test__generate_matrix_param__input_empty_name_value_dict__raise_exception(self):
        self.assertEqual(self.jj.generate_matrix_param({}, 'filter'), '')

    def test__generate_matrix_param__input_empty_value_list__raise_exception(self):
        name_value_test_dict = {'test_axis': []}
        self.assertRaises(Exception, self.jj.generate_matrix_param, name_value_test_dict, 'filter')

    def test__generate_matrix_param__input_list_istead_of_name_value_dict__raise_exception(self):
        self.assertRaises(AttributeError, self.jj.generate_matrix_param, ['test_value_1', 'test_value_2', 'test_value_3'], 'filter')

    def test__generate_matrix_param__input_empty_filter_string__return_matrix_config_string_without_filter(self):
        name_value_test_dict = {'test_axis': ['test_value_1', 'test_value_2', 'test_value_3']}
        self.assertTrue('combinationFilter' not in self.jj.generate_matrix_param(name_value_test_dict))

    # Testing generate_jointtrigger_param
    def test__generate_jointrigger_param__input_job_type_list_and_unstable_behavior_string__return_jointrigger_config_string(self):
        unstable_behavior_test = 'true'
        result = self.jj.generate_jointrigger_param(self.job_type_test_list, unstable_behavior_test)
        self.assertEqual(result, '<join.JoinTrigger> <joinProjects>test_user__pipe_starter, test_user__prio_build, test_user__normal_build</joinProjects> <joinPublishers/> <evenIfDownstreamUnstable>true</evenIfDownstreamUnstable> </join.JoinTrigger>')

    def test__generate_jointrigger_param__input_empty_job_type_list__return_empty_string(self):
        self.assertEqual(self.jj.generate_jointrigger_param([], 'true'), '')

    def test__generate_jointrigger_param__input_empty_job_type_list__return_empty_string2(self):
        self.assertEqual(self.jj.generate_jointrigger_param([], ''), '')

    def test__generate_jointrigger_param__input_empty_unstable_behavior_string__raise_exception(self):
        self.assertRaises(Exception, self.jj.generate_jointrigger_param, self.job_type_test_list, '')

    def test__generate_jointrigger_param__input_invalid_unstable_behavior_string__raise_exception(self):
        self.assertRaises(Exception, self.jj.generate_jointrigger_param, self.job_type_test_list, 'invalid')

    # Testing generate_postbuildtrigger_param
    def test__generate_postbuiltrigger_param__input_job_type_list_and_threshold_name_string__return_postbuildtrigger_config_string(self):
        treshold_name_test = 'SUCCESS'
        result = self.jj.generate_postbuildtrigger_param(self.job_type_test_list, treshold_name_test)
        self.assertEqual(result, '<hudson.tasks.BuildTrigger> <childProjects>test_user__pipe_starter, test_user__prio_build, test_user__normal_build</childProjects> <threshold> <name>SUCCESS</name> </threshold> </hudson.tasks.BuildTrigger>')

    def test__generate_postbuildtrigger_param__input_empty_job_type_list__return_empty_string(self):
        self.assertEqual(self.jj.generate_postbuildtrigger_param([], ''), '')

    def test__generate_postbuildtrigger_param__input_empty_job_type_list__return_empty_string2(self):
        self.assertEqual(self.jj.generate_postbuildtrigger_param([], 'SUCCESS'), '')

    def test__generate_postbuildtrigger_param__input_empty_threshold_name_string__raise_exception(self):
        self.assertRaises(Exception, self.jj.generate_postbuildtrigger_param, self.job_type_test_list, '')

    def test__generate_postbuildtrigger_param__input_invalid_threshold_name_string__raise_exception(self):
        self.assertRaises(Exception, self.jj.generate_postbuildtrigger_param, self.job_type_test_list, 'INVALID')

    # Testing generate_pipelinetrigger_param
    def test__generate_pipelinetrigger_param__input_job_type_list__return_pipelinetrigger_config_string(self):
        result = self.jj.generate_pipelinetrigger_param(self.job_type_test_list)
        self.assertEqual(result, '<au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger> <downstreamProjectNames>test_user__pipe_starter, test_user__prio_build, test_user__normal_build</downstreamProjectNames> </au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger>')

    def test__generate_pipelinetrigger_param__input_job_type_list__return_pipelinetrigger_config_string2(self):
        result = self.jj.generate_pipelinetrigger_param(['pipe'])
        self.assertEqual(result, '<au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger> <downstreamProjectNames>test_user__pipe_starter</downstreamProjectNames> </au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger>')

    def test__generate_pipelinetrigger_param__input_empty_job_type_list__return_empty_string(self):
        self.assertEqual(self.jj.generate_pipelinetrigger_param([]), '')

    # Testing generate_groovypostbuild_param
    def test__generate_groovypostbuild_param__input_script_type_string_project_list_behavior_string__return_groovypostbuild_config_string(self):
        result = self.jj.generate_groovypostbuild_param('enable', self.job_type_test_list, 2)
        self.assertEqual(result, "<org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder> <groovyScript>if(manager.build.result.isBetterOrEqualTo(hudson.model.Result.FAILURE)) { manager.listener.logger.println('Because this build did not fail:' for (project in ['test_user__pipe_starter', 'test_user__prio_build', 'test_user__normal_build']) { manager.listener.logger.println(' - ' + project) manager.hudson.getItem(project).enable() } manager.listener.logger.println('will be enabled.'}</groovyScript> <behavior>2</behavior> </org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder>")

    def test__generate_groovypostbuild_param__input_script_type_string_project_list_behavior_string__return_groovypostbuild_config_string2(self):
        result = self.jj.generate_groovypostbuild_param('disable', self.job_type_test_list, 2)
        self.assertEqual(result, "<org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder> <groovyScript>for (project in ['test_user__pipe_starter', 'test_user__prio_build', 'test_user__normal_build']) { manager.listener.logger.println('Disable ' + project + ' job') manager.hudson.getItem(project).disable() }</groovyScript> <behavior>2</behavior> </org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder>")

    def test__generate_groovypostbuild_param__input_empty_script_type_string__raise_exception(self):
        self.assertRaises(Exception, self.jj.generate_groovypostbuild_param, '', self.job_type_test_list, 2)

    def test__generate_groovypostbuild_param__input_invalid_script_type_string__raise_exception(self):
        self.assertRaises(KeyError, self.jj.generate_groovypostbuild_param, 'invalid', self.job_type_test_list, 2)

    def test__generate_groovypostbuild_param__input_empty_project_list__raise_exception(self):
        self.assertRaises(Exception, self.jj.generate_groovypostbuild_param, 'enable', [], 2)

    def test__generate_groovypostbuild_param__input_invalid_project_list_entry__raise_exception(self):
        self.assertRaises(Exception, self.jj.generate_groovypostbuild_param, 'enable', ['invalid'], 2)

    def test__generate_groovypostbuild_param__input_invalid_behavior__raise_exception(self):
        self.assertRaises(Exception, self.jj.generate_groovypostbuild_param, 'enable', self.job_type_test_list, 3)

    def test__generate_groovypostbuild_param__input_invalid_behavior__raise_exception2(self):
        self.assertRaises(Exception, self.jj.generate_groovypostbuild_param, 'enable', self.job_type_test_list, -1)

    def test__generate_groovypostbuild_param__input_invalid_behavior__raise_exception3(self):
        self.assertRaises(Exception, self.jj.generate_groovypostbuild_param, 'enable', self.job_type_test_list, '1')

    # Testing common_params
    def test__get_common_params__input_job_type_string__return_common_job_config_dict(self):
        common_job_config_dict = {'USERNAME': 'test_user',
                                  'EMAIL': 'test@ipa.fhg.de',
                                  'EMAIL_COMMITTER': 'false',
                                  'JOB_TYPE_NAME': 'pipe_starter',
                                  'SCRIPT': 'pipe_starter',
                                  'NODE_LABEL': 'pipe',
                                  'TIME': datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d %H:%M'),
                                  'HOSTNAME': socket.gethostname(),
                                  'ROSDISTRO': ['test_rosdistro'],
                                  'PROJECT': 'matrix-project',
                                  'TRIGGER': '<triggers class="vector"/>',
                                  'VCS': '<scm class="hudson.scm.NullSCM"/>',
                                  'MATRIX': '',
                                  'PARAMETERS': '',
                                  'POSTBUILD_TRIGGER': '',
                                  'JOIN_TRIGGER': '',
                                  'PIPELINE_TRIGGER': '',
                                  'GROOVY_POSTBUILD': ''
                                  }
        result = self.jj.get_common_params('pipe')
        self.assertEqual(result, common_job_config_dict)

    def test__get_common_params__input_empty_job_type_string__raise_exception(self):
        self.assertRaises(Exception, self.jj.get_common_params, '')

    def test__get_common_params__input_invalid_job_type_string__raise_exception(self):
        self.assertRaises(Exception, self.jj.get_common_params, 'invalid')

    # Testing generate_prio_distro_arch
    def test__generate_prio_distro_arch__return_prio_list(self):
        result = self.jj.generate_prio_distro_arch()
        self.assertEqual(result, [{'ros_distro': 'test_rosdistro',
                                   'ubuntu_distro': 'natty',
                                   'arch': 'amd64'}])

    def test__generate_prio_distro_arch__return_prio_list2(self):
        self.jj.PRIO_DISTRO['test_rosdistro2'] = 'precise'
        self.jj.TARGET_PLATFORM['test_rosdistro2'] = ['lucid', 'oneiric', 'precise']
        self.jj.pipe_conf['ros_distro'].append('test_rosdistro2')
        self.jj.pipe_conf['prio_distro_arch']['test_rosdistro2'] = {'ubuntu_distro': 'oneiric',
                                                                    'arch': 'i386'}
        result = self.jj.generate_prio_distro_arch()
        self.assertEqual(result, [{'ros_distro': 'test_rosdistro',
                                   'ubuntu_distro': 'natty',
                                   'arch': 'amd64'},
                                  {'ros_distro': 'test_rosdistro2',
                                   'ubuntu_distro': 'oneiric',
                                   'arch': 'i386'}])

    # Testing generate_distro_arch_field
    def test__generate_distro_arch_field__return_name_value_dict(self):
        result = self.jj.generate_distro_arch_field()
        self.assertEqual(result, {'ros_distro': ['test_rosdistro'],
                                  'ubuntu_distro': ['natty', 'oneiric', 'precise'],
                                  'arch': ['i386', 'amd64']})

    def test__generate_distro_arch_field__return_name_value_dict2(self):
        self.jj.PRIO_DISTRO['test_rosdistro2'] = 'precise'
        self.jj.TARGET_PLATFORM['test_rosdistro2'] = ['lucid', 'oneiric', 'precise']
        self.jj.pipe_conf['ros_distro'].append('test_rosdistro2')
        self.jj.pipe_conf['prio_distro_arch']['test_rosdistro2'] = {'ubuntu_distro': 'oneiric',
                                                                    'arch': 'i386'}
        result = self.jj.generate_distro_arch_field()
        self.assertEqual(result, {'ros_distro': ['test_rosdistro', 'test_rosdistro2'],
                                  'ubuntu_distro': ['natty', 'oneiric', 'precise', 'lucid'],
                                  'arch': ['i386', 'amd64']})

    # Testing generate_matrix_filter
    def test__generate_matrix_filter__input_dict_list_and_negation_boolean_return_filter_string(self):
        test_dict = [{'ros_distro': 'test_rosdistro',
                      'ubuntu_distro': 'natty',
                      'arch': 'amd64'}]
        result = self.jj.generate_matrix_filter(test_dict, False)
        self.assertEqual(result, '((ubuntu_distro == natty && arch == amd64 && ros_distro == test_rosdistro))')

    def test__generate_matrix_filter__input_dict_list_and_negation_boolean_return_filter_string2(self):
        test_dict = [{'ros_distro': 'test_rosdistro',
                      'ubuntu_distro': 'natty',
                      'arch': 'amd64'}]
        result = self.jj.generate_matrix_filter(test_dict, True)
        self.assertEqual(result, '!((ubuntu_distro == natty && arch == amd64 && ros_distro == test_rosdistro))')

    def test__generate_matrix_filter__input_dict_list_and_negation_boolean_return_filter_string3(self):
        test_dict = [{'ros_distro': 'test_rosdistro'},
                     {'ubuntu_distro': 'natty',
                      'arch': 'amd64'}]
        result = self.jj.generate_matrix_filter(test_dict, False)
        self.assertEqual(result, '((ros_distro == test_rosdistro) || (ubuntu_distro == natty && arch == amd64))')

    def test__generate_matrix_filter__input_dict_list_and_negation_boolean_return_filter_string4(self):
        test_dict = [{'ros_distro': 'test_rosdistro'},
                     {'ubuntu_distro': 'natty',
                      'arch': 'amd64'}]
        result = self.jj.generate_matrix_filter(test_dict, True)
        self.assertEqual(result, '!((ros_distro == test_rosdistro) || (ubuntu_distro == natty && arch == amd64))')

    # Testing pipe_starter_params
    def test__pipe_starter_params__input_params_dict__return_reworked_params_dict(self):
        pipe_starter_config_dict = {'NODE_LABEL': 'master',
                                    'PROJECT': 'project',
                                    'TRIGGER': '<triggers class="vector"> <hudson.triggers.SCMTrigger> <spec>*/10 * * * *</spec> </hudson.triggers.SCMTrigger> </triggers>',
                                    'VCS': '<scm class="hudson.plugins.git.GitSCM"> <configVersion>2</configVersion> <userRemoteConfigs> <hudson.plugins.git.UserRemoteConfig> <name>origin</name> <refspec>+refs/heads/master:refs/remotes/origin/master</refspec> <url>git@github.com/user/repo1</url> </hudson.plugins.git.UserRemoteConfig> <hudson.plugins.git.UserRemoteConfig> <name>origin</name> <refspec>+refs/heads/test:refs/remotes/origin/test</refspec> <url>git@github.com/user/repo2</url> </hudson.plugins.git.UserRemoteConfig> </userRemoteConfigs> <branches> <hudson.plugins.git.BranchSpec> <name>''</name> </hudson.plugins.git.BranchSpec> </branches> <disableSubmodules>false</disableSubmodules> <recursiveSubmodules>true</recursiveSubmodules> <doGenerateSubmoduleConfigurations>false</doGenerateSubmoduleConfigurations> <authorOrCommitter>false</authorOrCommitter> <clean>false</clean> <wipeOutWorkspace>false</wipeOutWorkspace> <pruneBranches>false</pruneBranches> <remotePoll>false</remotePoll> <ignoreNotifyCommit>false</ignoreNotifyCommit> <buildChooser class="hudson.plugins.git.util.DefaultBuildChooser"/> <gitTool>Default</gitTool> <submoduleCfg class="list"/> <relativeTargetDir>monitored_vcs</relativeTargetDir> <reference/> <excludedRegions/> <excludedUsers/> <gitConfigName/> <gitConfigEmail/> <skipTag>false</skipTag> <includedRegions/> <scmName/> </scm>',
                                    'GROOVY_POSTBUILD': "<org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder> <groovyScript>for (project in ['test_user__bringup_hardware_test', 'test_user__highlevel_hardware_test', 'test_user__release_params']) { manager.listener.logger.println('Disable ' + project + ' job') manager.hudson.getItem(project).disable() }</groovyScript> <behavior>2</behavior> </org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder>",
                                    'POSTBUILD_TRIGGER': '<hudson.tasks.BuildTrigger> <childProjects>test_user__prio_build</childProjects> <threshold> <name>SUCCESS</name> </threshold> </hudson.tasks.BuildTrigger>'
                                    }
        result = self.jj.pipe_starter_params({})
        self.assertEqual(result, pipe_starter_config_dict)

    # Testing prio_build_params TODO
    def test__prio_build_params__input_params_dict__return_reworked_params_dict(self):
        prio_build_config_dict = {'NODE_LABEL': 'prio_build',
                                  'MATRIX': '<axes> <hudson.matrix.TextAxis> <name>ubuntu_distro</name> <values> <string>natty</string> <string>oneiric</string> <string>precise</string> </values> </hudson.matrix.TextAxis> <hudson.matrix.TextAxis> <name>arch</name> <values> <string>i386</string> <string>amd64</string> </values> </hudson.matrix.TextAxis> <hudson.matrix.TextAxis> <name>ros_distro</name> <values> <string>test_rosdistro</string> </values> </hudson.matrix.TextAxis> </axes> <combinationFilter>((ubuntu_distro == natty && arch == amd64 && ros_distro == test_rosdistro))</combinationFilter>',
                                  'SCRIPT': 'prio_build_script',  # TODO
                                  'GROOVY_POSTBUILD': "<org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder> <groovyScript>if(manager.build.result.isBetterOrEqualTo(hudson.model.Result.FAILURE)) { manager.listener.logger.println('Because this build did not fail:' for (project in ['test_user__bringup_hardware_test']) { manager.listener.logger.println(' - ' + project) manager.hudson.getItem(project).enable() } manager.listener.logger.println('will be enabled.'}</groovyScript> <behavior>2</behavior> </org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder>",
                                  'POSTBUILD_TRIGGER': '<hudson.tasks.BuildTrigger> <childProjects>test_user__downstream_build</childProjects> <threshold> <name>SUCCESS</name> </threshold> </hudson.tasks.BuildTrigger>',
                                  'PIPELINE_TRIGGER': '<au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger> <downstreamProjectNames>test_user__bringup_hardware_test</downstreamProjectNames> </au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger>',
                                  }
        result = self.jj.prio_build_params({})
        self.assertEqual(result, prio_build_config_dict)

    # Testing normal_build_params TODO
    def test__normal_build_params__input_params_dict__return_reworked_params_dict(self):
        normal_build_config_dict = {'NODE_LABEL': 'build',
                                    'MATRIX': '<axes> <hudson.matrix.TextAxis> <name>ubuntu_distro</name> <values> <string>natty</string> <string>oneiric</string> <string>precise</string> </values> </hudson.matrix.TextAxis> <hudson.matrix.TextAxis> <name>arch</name> <values> <string>i386</string> <string>amd64</string> </values> </hudson.matrix.TextAxis> <hudson.matrix.TextAxis> <name>ros_distro</name> <values> <string>test_rosdistro</string> </values> </hudson.matrix.TextAxis> </axes> <combinationFilter>!((ubuntu_distro == natty && arch == amd64 && ros_distro == test_rosdistro))</combinationFilter>'}
        result = self.jj.normal_build_params({})
        self.assertEqual(result, normal_build_config_dict)

    # Testing downstream_build_params TODO
    def test__downstream_build_params__input_params_dict__return_reworked_params_dict(self):
        downstream_build_config_dict = {'NODE_LABEL': 'build',
                                        'POSTBUILD_TRIGGER': '<hudson.tasks.BuildTrigger> <childProjects>test_user__database_test, test_user__simulation_test</childProjects> <threshold> <name>SUCCESS</name> </threshold> </hudson.tasks.BuildTrigger>',
                                        'JOIN_TRIGGER': '<join.JoinTrigger> <joinProjects>test_user__application_test</joinProjects> <joinPublishers/> <evenIfDownstreamUnstable>true</evenIfDownstreamUnstable> </join.JoinTrigger>',
                                        }
        result = self.jj.downstream_buils_params({})
        self.assertEqual(result, downstream_build_config_dict)

    # Testing get_job_type_params
    def test__get_job_type_params__input_job_type_string_and_params_dict_return_reworked_params_dict(self):
        pipe_starter_config_dict = {'NODE_LABEL': 'master',
                                    'PROJECT': 'project',
                                    'TRIGGER': '<triggers class="vector"> <hudson.triggers.SCMTrigger> <spec>*/10 * * * *</spec> </hudson.triggers.SCMTrigger> </triggers>',
                                    'VCS': '<scm class="hudson.plugins.git.GitSCM"> <configVersion>2</configVersion> <userRemoteConfigs> <hudson.plugins.git.UserRemoteConfig> <name>origin</name> <refspec>+refs/heads/master:refs/remotes/origin/master</refspec> <url>git@github.com/user/repo1</url> </hudson.plugins.git.UserRemoteConfig> <hudson.plugins.git.UserRemoteConfig> <name>origin</name> <refspec>+refs/heads/test:refs/remotes/origin/test</refspec> <url>git@github.com/user/repo2</url> </hudson.plugins.git.UserRemoteConfig> </userRemoteConfigs> <branches> <hudson.plugins.git.BranchSpec> <name>''</name> </hudson.plugins.git.BranchSpec> </branches> <disableSubmodules>false</disableSubmodules> <recursiveSubmodules>true</recursiveSubmodules> <doGenerateSubmoduleConfigurations>false</doGenerateSubmoduleConfigurations> <authorOrCommitter>false</authorOrCommitter> <clean>false</clean> <wipeOutWorkspace>false</wipeOutWorkspace> <pruneBranches>false</pruneBranches> <remotePoll>false</remotePoll> <ignoreNotifyCommit>false</ignoreNotifyCommit> <buildChooser class="hudson.plugins.git.util.DefaultBuildChooser"/> <gitTool>Default</gitTool> <submoduleCfg class="list"/> <relativeTargetDir>monitored_vcs</relativeTargetDir> <reference/> <excludedRegions/> <excludedUsers/> <gitConfigName/> <gitConfigEmail/> <skipTag>false</skipTag> <includedRegions/> <scmName/> </scm>',
                                    'GROOVY_POSTBUILD': "<org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder> <groovyScript>for (project in ['test_user__bringup_hardware_test', 'test_user__highlevel_hardware_test', 'test_user__release_params']) { manager.listener.logger.println('Disable ' + project + ' job') manager.hudson.getItem(project).disable() }</groovyScript> <behavior>2</behavior> </org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder>",
                                    'POSTBUILD_TRIGGER': '<hudson.tasks.BuildTrigger> <childProjects>test_user__prio_build</childProjects> <threshold> <name>SUCCESS</name> </threshold> </hudson.tasks.BuildTrigger>'
                                    }
        result = self.jj.get_job_type_params('pipe', {})
        self.assertEqual(result, pipe_starter_config_dict)

    # Testing schedule_job
    def test__schedule_job__input_job_name_string_and_job_config_string__return_action_string(self):
        test_job_config = jenkins.EMPTY_CONFIG_XML
        self.jj.delete_job('test_job')
        result = self.jj.schedule_job('test_job', test_job_config)
        self.assertEqual(result, 'created')
        self.jj.delete_job('test_job')

    def test__schedule_job__input_job_name_string_and_job_config_string__return_action_string2(self):
        test_job_config = jenkins.EMPTY_CONFIG_XML
        self.jj.schedule_job('test_job', test_job_config)
        result = self.jj.schedule_job('test_job', test_job_config)
        self.assertEqual(result, 'reconfigured')
        self.jj.delete_job('test_job')

    # Testing delete_job
    def test__delete_job__input_job_name_string__return_action_string(self):
        test_job_config = jenkins.EMPTY_CONFIG_XML
        self.jj.schedule_job('test_job', test_job_config)
        result = self.jj.delete_job('test_job')
        self.assertEqual(result, 'deleted')

    def test__delete_job__input_job_name_string__return_action_string2(self):
        result = self.jj.delete_job('test_job')
        self.assertEqual(result, 'not existent')


if __name__ == '__main__':
    unittest.main()
