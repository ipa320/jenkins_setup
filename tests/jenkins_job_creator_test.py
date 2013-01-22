#!/usr/bin/env python

import unittest

import os
import datetime
import socket
import yaml
import jenkins

from jenkins_setup import jenkins_job_creator, cob_common


class Jenkins_Job_Test(unittest.TestCase):
    """
    Tests Jenkins jobs
    """

    def setUp(self):
        self.maxDiff = None

        self.test_dict = cob_common.get_buildpipeline_configs('jenkins-test-server', 'test-user')

        self.job_type_test_list = ['pipe', 'prio', 'normal']

        with open(os.path.expanduser('~/jenkins-config/slave_config.yaml')) as f:
            info = yaml.load(f)
        jenkins_instance = jenkins.Jenkins(info['master_url'], info['jenkins_login'], info['jenkins_pw'])

        self.jj = jenkins_job_creator.Jenkins_Job(jenkins_instance, self.test_dict)



    # Testing common_params
    def test__get_common_params__return_common_job_config_dict(self):
        self.jj.job_type = 'pipe'
        common_job_config_dict = {'USERNAME': 'test-user',
                                  'EMAIL': 'test@ipa.fhg.de',
                                  'EMAIL_COMMITTER': 'false',
                                  'JOB_TYPE_NAME': 'pipe_starter',
                                  'SCRIPT': 'pipe_starter',
                                  'NODE_LABEL': 'pipe',
                                  'TIME': datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d %H:%M'),
                                  'HOSTNAME': socket.gethostname(),
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
        self.jj.get_common_params()
        self.assertEqual(self.jj.params, common_job_config_dict)

    def test__get_common_params__empty_job_type_string__raise_exception(self):
        self.jj.job_type = ''
        self.assertRaises(Exception, self.jj.get_common_params)

    def test__get_common_params__invalid_job_type_string__raise_exception(self):
        self.jj.job_type = 'invalid'
        self.assertRaises(Exception, self.jj.get_common_params)

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

    # Testing generate_job_name
    def test__generate_job_name__input_job_string__return_job_name_string(self):
        result = self.jj.generate_job_name('pipe')
        self.assertEqual(result, 'test-user__pipe_starter')

    def test__generate_job_name__input_job_string__return_job_name_string2(self):
        result = self.jj.generate_job_name('hilevel')
        self.assertEqual(result, 'test-user__highlevel_hardware_test')

    def test__generate_job_name__input_job_and_suffix_string__return_job_name_string(self):
        result = self.jj.generate_job_name('pipe', 'suffix')
        self.assertEqual(result, 'test-user__pipe_starter__suffix')

    def test__generate_job_name__input_wrong_key_string__raise_exception(self):
        self.assertRaises(KeyError, self.jj.generate_job_name, 'wrong_key')

    def test__generate_job_name__input_wrong_key_int__raise_exception(self):
        self.assertRaises(KeyError, self.jj.generate_job_name, 12)

    # Testing generate_job_list
    def test__generate_job_list__input_job_list__return_job_name_list(self):
        result = self.jj.generate_job_list(self.job_type_test_list)
        self.assertEqual(result, ['test-user__pipe_starter',
                                  'test-user__prio_build',
                                  'test-user__normal_build'])

    def test__generate_job_list__input_empty_list__return_empty_string(self):
        result = self.jj.generate_job_list([])
        self.assertEqual(result, [])

    def test__generate_job_list__input_dict__raise_exception(self):
        self.assertRaises(TypeError, self.jj.generate_job_list, ('pipe', 'prio', 'normal'))

    # Testing generate_job_list_string
    def test__generate_job_list_string__input_job_list__return_job_names_as_comma_separatet_string(self):
        result = self.jj.generate_job_list_string(self.job_type_test_list)
        self.assertEqual(result, 'test-user__pipe_starter, test-user__prio_build, test-user__normal_build')

    def test__generate_job_list_string__input_empty_list__return_empty_string(self):
        result = self.jj.generate_job_list_string([])
        self.assertEqual(result, '')

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
        self.assertEqual(result, '<join.JoinTrigger> <joinProjects>test-user__pipe_starter, test-user__prio_build, test-user__normal_build</joinProjects> <joinPublishers/> <evenIfDownstreamUnstable>true</evenIfDownstreamUnstable> </join.JoinTrigger>')

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
        self.assertEqual(result, '<hudson.tasks.BuildTrigger> <childProjects>test-user__pipe_starter, test-user__prio_build, test-user__normal_build</childProjects> <threshold> <name>SUCCESS</name> </threshold> </hudson.tasks.BuildTrigger>')

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
        self.assertEqual(result, '<au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger> <downstreamProjectNames>test-user__pipe_starter, test-user__prio_build, test-user__normal_build</downstreamProjectNames> </au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger>')

    def test__generate_pipelinetrigger_param__input_job_type_list__return_pipelinetrigger_config_string2(self):
        result = self.jj.generate_pipelinetrigger_param(['pipe'])
        self.assertEqual(result, '<au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger> <downstreamProjectNames>test-user__pipe_starter</downstreamProjectNames> </au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger>')

    def test__generate_pipelinetrigger_param__input_empty_job_type_list__return_empty_string(self):
        self.assertEqual(self.jj.generate_pipelinetrigger_param([]), '')

    # Testing generate_groovypostbuild_param
    def test__generate_groovypostbuild_param__input_script_type_string_project_list_behavior_string__return_groovypostbuild_config_string(self):
        result = self.jj.generate_groovypostbuild_param('enable', self.job_type_test_list, 2)
        self.assertEqual(result, "<org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder> <groovyScript>if(manager.build.result.isBetterOrEqualTo(hudson.model.Result.FAILURE)) { manager.listener.logger.println('Because this build did not fail:' for (project in ['test-user__pipe_starter', 'test-user__prio_build', 'test-user__normal_build']) { manager.listener.logger.println(' - ' + project) manager.hudson.getItem(project).enable() } manager.listener.logger.println('will be enabled.'}</groovyScript> <behavior>2</behavior> </org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder>")

    def test__generate_groovypostbuild_param__input_script_type_string_project_list_behavior_string__return_groovypostbuild_config_string2(self):
        result = self.jj.generate_groovypostbuild_param('disable', self.job_type_test_list, 2)
        self.assertEqual(result, "<org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder> <groovyScript>for (project in ['test-user__pipe_starter', 'test-user__prio_build', 'test-user__normal_build']) { manager.listener.logger.println('Disable ' + project + ' job') manager.hudson.getItem(project).disable() }</groovyScript> <behavior>2</behavior> </org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder>")

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
