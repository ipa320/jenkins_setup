#!/usr/bin/env python

import unittest

import os
import datetime
import socket
import yaml
import jenkins

from jenkins_setup import jenkins_job_creator, cob_pipe


class Jenkins_Job_Test(unittest.TestCase):
    """
    Tests Jenkins jobs
    """

    def setUp(self):
        self.maxDiff = None

        #self.test_dict = cob_common.get_buildpipeline_configs('jenkins-test-server', 'test-user')
        self.test_pipe_inst = cob_pipe.Cob_Pipe()
        self.test_pipe_inst.load_config_from_url('jenkins-test-server', 'test-user')

        self.job_type_test_list = ['pipe', 'prio', 'normal']

        with open(os.path.expanduser('~/jenkins-config/slave_config.yaml')) as f:
            info = yaml.load(f)
        jenkins_instance = jenkins.Jenkins(info['master_url'], info['jenkins_login'], info['jenkins_pw'])

        self.jj = jenkins_job_creator.Jenkins_Job(jenkins_instance, self.test_pipe_inst)

    # Testing schedule_job
    def test__schedule_job__job_name_string_and_job_config_string__return_action_string(self):
        self.jj.job_name = 'test_job'
        self.jj.job_config = jenkins.EMPTY_CONFIG_XML
        self.jj.delete_job()
        result = self.jj.schedule_job()
        self.assertEqual(result, 'created')
        self.jj.delete_job()

    def test__schedule_job__job_name_string_and_job_config_string__return_action_string2(self):
        self.jj.job_name = 'test_job'
        self.jj.job_config = jenkins.EMPTY_CONFIG_XML
        self.jj.schedule_job()
        result = self.jj.schedule_job()
        self.assertEqual(result, 'reconfigured')
        self.jj.delete_job()

    # Testing delete_job
    def test__delete_job__job_name_string__return_action_string(self):
        self.jj.job_name = 'test_job'
        self.jj.job_config = jenkins.EMPTY_CONFIG_XML
        self.jj.schedule_job()
        result = self.jj.delete_job()
        self.assertEqual(result, 'test_job')

    def test__delete_job__job_name_string__return_action_string2(self):
        self.jj.job_name = 'test_job'
        result = self.jj.delete_job()
        self.assertEqual(result, '')

    # Testing common_params
    def test__set_common_params__return_common_job_config_dict(self):
        self.jj.job_type = 'pipe'
        common_job_config_dict = {'SHELL': '',
                                  'USERNAME': 'test-user',
                                  #'JOB_TYPE_NAME': 'pipe_starter',
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
                                  'GROOVY_POSTBUILD': '',
                                  'PARAMETERIZED_TRIGGER': '',
                                  'JUNIT_TESTRESULTS': '',
                                  'MAILER': ''
                                  }
        self.jj.set_common_params()
        self.assertEqual(self.jj.params, common_job_config_dict)

    def test__set_common_params__empty_job_type_string__raise_exception(self):
        self.jj.job_type = ''
        self.assertRaises(Exception, self.jj.set_common_params)

    def test__set_common_params__invalid_job_type_string__raise_exception(self):
        self.jj.job_type = 'invalid'
        self.assertRaises(Exception, self.jj.set_common_params)

    # Testing replace_placeholder
    def test__replace_placeholder__set_config_string_and_params_dict__check_config_string(self):
        self.jj.job_config = '@(TEST_1) and @(TEST_2) should be replaced'
        self.jj.params = {'TEST_1': 'THIS', 'TEST_2': 'THAT'}
        self.jj.replace_placeholder()
        self.assertEqual(self.jj.job_config, 'THIS and THAT should be replaced')

    def test__replace_placeholder__set_not_existent_key__raise_exception(self):
        self.jj.job_config = '@(TEST_1) and @(TEST_2) should be replaced'
        self.jj.params = {'WRONG': 'THIS', 'TEST_2': 'THAT'}
        self.assertRaises(KeyError, self.jj.replace_placeholder)

    def test__replace_placeholder__missing_parameter__raise_exception(self):
        self.jj.job_config = '@(TEST_1) and @(TEST_2) should be replaced'
        self.jj.params = {'TEST_1': 'THIS'}
        self.assertRaises(KeyError, self.jj.replace_placeholder)

    def test__replace_placeholder__missing_parameter__raise_exception2(self):
        self.jj.job_config = '@(TEST_1) and @(TEST_2) should be @( replaced'
        self.jj.params = {'TEST_1': 'THIS', 'TEST_2': 'THAT'}
        self.jj.replace_placeholder()
        self.assertEqual(self.jj.job_config, 'THIS and THAT should be @( replaced')

    # Testing generate_job_name
    def test__generate_job_name__input_job_string__return_job_name_string(self):
        result = self.jj.generate_job_name('pipe')
        self.assertEqual(result, 'test-user__pipe_starter')

    def test__generate_job_name__input_job_string__return_job_name_string2(self):
        result = self.jj.generate_job_name('highlevel')
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
        self.assertEqual(result, '(ubuntu_distro=="natty" &amp;&amp; arch=="amd64" &amp;&amp; ros_distro=="test_rosdistro")')

    def test__generate_matrix_filter__input_dict_list_and_negation_boolean_return_filter_string2(self):
        test_dict = [{'ros_distro': 'test_rosdistro',
                      'ubuntu_distro': 'natty',
                      'arch': 'amd64'}]
        result = self.jj.generate_matrix_filter(test_dict, True)
        self.assertEqual(result, '!((ubuntu_distro=="natty" &amp;&amp; arch=="amd64" &amp;&amp; ros_distro=="test_rosdistro"))')

    def test__generate_matrix_filter__input_dict_list_and_negation_boolean_return_filter_string3(self):
        test_dict = [{'ros_distro': 'test_rosdistro'},
                     {'ubuntu_distro': 'natty',
                      'arch': 'amd64'}]
        result = self.jj.generate_matrix_filter(test_dict, False)
        self.assertEqual(result, '(ros_distro=="test_rosdistro") || (ubuntu_distro=="natty" &amp;&amp; arch=="amd64")')

    def test__generate_matrix_filter__input_dict_list_and_negation_boolean_return_filter_string4(self):
        test_dict = [{'ros_distro': 'test_rosdistro'},
                     {'ubuntu_distro': 'natty',
                      'arch': 'amd64'}]
        result = self.jj.generate_matrix_filter(test_dict, True)
        self.assertEqual(result, '!((ros_distro=="test_rosdistro") || (ubuntu_distro=="natty" &amp;&amp; arch=="amd64"))')

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

    # Testing set_matrix_param
    def test__set_matrix_param__input_name_value_dict_list_and_filter_string__return_matrix_config_string(self):
        name_value_test_dict = [{'test_axis': ['test_value_1', 'test_value_2', 'test_value_3']}]
        self.jj.set_matrix_param(name_value_test_dict, 'filter')
        self.assertEqual(self.jj.params['MATRIX'], '<axes> <hudson.matrix.TextAxis> <name>test_axis</name> <values> <string>test_value_1</string> <string>test_value_2</string> <string>test_value_3</string> </values> </hudson.matrix.TextAxis> </axes> <combinationFilter>filter</combinationFilter>')

    def test__set_matrix_param__input_name_value_dict_and_filter_string__return_matrix_config_string2(self):
        name_value_test_dict = [{'test_axis': ['test_value_1', 'test_value_2', 'test_value_3']}]
        self.jj.set_matrix_param(name_value_test_dict, 'filter')
        self.assertEqual(self.jj.params['MATRIX'], '<axes> <hudson.matrix.TextAxis> <name>test_axis</name> <values> <string>test_value_1</string> <string>test_value_2</string> <string>test_value_3</string> </values> </hudson.matrix.TextAxis> </axes> <combinationFilter>filter</combinationFilter>')

    def test__set_matrix_param__input_empty_list_raise_exception(self):
        self.assertEqual(self.jj.set_matrix_param([], 'filter'), '')

    def test__set_matrix_param__input_empty_name_value_dict__raise_exception(self):
        self.assertEqual(self.jj.set_matrix_param([{}], 'filter'), '')

    def test__set_matrix_param__input_empty_name_value_dict__raise_exception2(self):
        name_value_test_dict = [{}, {'test_axis': ['test_value_1', 'test_value_2', 'test_value_3']}]
        self.jj.set_matrix_param(name_value_test_dict, 'filter')
        self.assertEqual(self.jj.params['MATRIX'], '<axes> <hudson.matrix.TextAxis> <name>test_axis</name> <values> <string>test_value_1</string> <string>test_value_2</string> <string>test_value_3</string> </values> </hudson.matrix.TextAxis> </axes> <combinationFilter>filter</combinationFilter>')

    def test__set_matrix_param__input_empty_value_list__raise_exception(self):
        name_value_test_dict = [{'test_axis': []}]
        self.assertRaises(Exception, self.jj.set_matrix_param, name_value_test_dict, 'filter')

    def test__set_matrix_param__input_list_instead_of_name_value_dict__raise_exception(self):
        self.assertRaises(AttributeError, self.jj.set_matrix_param, ['test_value_1', 'test_value_2', 'test_value_3'], 'filter')

    def test__set_matrix_param__input_empty_filter_string__return_matrix_config_string_without_filter(self):
        name_value_test_dict = [{'test_axis': ['test_value_1', 'test_value_2', 'test_value_3']}]
        self.jj.set_matrix_param(name_value_test_dict)
        self.assertTrue('combinationFilter' not in self.jj.params['MATRIX'])

    # Testing get_matrix_entries
    def test__get_matrix_entries__result_dict_list(self):
        result = self.jj.get_matrix_entries()
        self.assertEqual(result, [{'repository': ['test_repo_1', 'test_repo_3', 'test_repo_2', 'test_repo_4']},
                                  {'ros_distro': ['test_rosdistro', 'test_rosdistro_2']},
                                  {'ubuntu_distro': ['oneiric', 'lucid', 'natty']},
                                  {'arch': ['amd64', 'i386']}])

    # Testing set_jointrigger_param
    def test__set_jointrigger_param__input_job_type_list_and_unstable_behavior_bool__check_set_param(self):
        unstable_behavior_test = True
        self.jj.set_jointrigger_param(self.job_type_test_list, unstable_behavior_test)
        self.assertEqual(self.jj.params['JOIN_TRIGGER'], '<join.JoinTrigger> <joinProjects>test-user__pipe_starter, test-user__prio_build, test-user__normal_build</joinProjects> <joinPublishers>  </joinPublishers> <evenIfDownstreamUnstable>true</evenIfDownstreamUnstable> </join.JoinTrigger>')

    def test__set_jointrigger_param__input_job_type_list_and_unstable_behavior_bool__check_set_param2(self):
        unstable_behavior_test = False
        self.jj.set_jointrigger_param(self.job_type_test_list, unstable_behavior_test)
        self.assertEqual(self.jj.params['JOIN_TRIGGER'], '<join.JoinTrigger> <joinProjects>test-user__pipe_starter, test-user__prio_build, test-user__normal_build</joinProjects> <joinPublishers>  </joinPublishers> <evenIfDownstreamUnstable>false</evenIfDownstreamUnstable> </join.JoinTrigger>')

    def test__set_jointrigger_param__input_job_type_list_and_unstable_behavior_bool_and_parameterized_trigger_str__check_set_param(self):
        parameterized_trigger_test = 'TESTCONFIG'
        unstable_behavior_test = False
        self.jj.set_jointrigger_param(self.job_type_test_list, unstable_behavior_test, parameterized_trigger_test)
        self.assertEqual(self.jj.params['JOIN_TRIGGER'], '<join.JoinTrigger> <joinProjects>test-user__pipe_starter, test-user__prio_build, test-user__normal_build</joinProjects> <joinPublishers> TESTCONFIG </joinPublishers> <evenIfDownstreamUnstable>false</evenIfDownstreamUnstable> </join.JoinTrigger>')

    def test__set_jointrigger_param__input_empty_job_type_list__check_set_param(self):
        self.jj.set_jointrigger_param([], False, 'TESTCONFIG')
        self.assertEqual(self.jj.params['JOIN_TRIGGER'], '<join.JoinTrigger> <joinProjects></joinProjects> <joinPublishers> TESTCONFIG </joinPublishers> <evenIfDownstreamUnstable>false</evenIfDownstreamUnstable> </join.JoinTrigger>')

    def test__set_jointrigger_param__input_no_job_type_list_and_empty_parameterized_trigger_str__raise_exception(self):
        self.assertRaises(Exception, self.jj.set_jointrigger_param, [], parameterized_trigger='')

    def test__set_jointrigger_param__input_no_job_type_list_and_no_parameterized_trigger_str__raise_exception(self):
        self.assertRaises(Exception, self.jj.set_jointrigger_param, [], parameterized_trigger=None)

    def test__set_jointrigger_param__input_unstable_behavior_not_bool__raise_exception(self):
        self.assertRaises(Exception, self.jj.set_jointrigger_param, self.job_type_test_list, unstable_behavior_test='False', parameterized_trigger='TESTCONFIG')

    # Testing set_postbuildtrigger_param
    def test__set_postbuildtrigger_param__input_job_type_list_and_threshold_name_string__return_postbuildtrigger_config_string(self):
        treshold_name_test = 'SUCCESS'
        self.jj.set_postbuildtrigger_param(self.job_type_test_list, treshold_name_test)
        self.assertEqual(self.jj.params['POSTBUILD_TRIGGER'], '<hudson.tasks.BuildTrigger> <childProjects>test-user__pipe_starter, test-user__prio_build, test-user__normal_build</childProjects> <threshold> <name>SUCCESS</name> </threshold> </hudson.tasks.BuildTrigger>')

    def test__set_postbuildtrigger_param__input_empty_job_type_list__return_empty_string(self):
        self.assertEqual(self.jj.set_postbuildtrigger_param([], ''), '')

    def test__set_postbuildtrigger_param__input_empty_job_type_list__return_empty_string2(self):
        self.assertEqual(self.jj.set_postbuildtrigger_param([], 'SUCCESS'), '')

    def test__set_postbuildtrigger_param__input_empty_threshold_name_string__raise_exception(self):
        self.assertRaises(Exception, self.jj.set_postbuildtrigger_param, self.job_type_test_list, '')

    def test__set_postbuildtrigger_param__input_invalid_threshold_name_string__raise_exception(self):
        self.assertRaises(Exception, self.jj.set_postbuildtrigger_param, self.job_type_test_list, 'INVALID')

    # Testing set_pipelinetrigger_param
    def test__set_pipelinetrigger_param__input_job_type_list__return_pipelinetrigger_config_string(self):
        self.jj.set_pipelinetrigger_param(self.job_type_test_list)
        self.assertEqual(self.jj.params['PIPELINE_TRIGGER'], '<au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger> <downstreamProjectNames>test-user__pipe_starter, test-user__prio_build, test-user__normal_build</downstreamProjectNames> </au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger>')

    def test__set_pipelinetrigger_param__input_job_type_list__return_pipelinetrigger_config_string2(self):
        self.jj.set_pipelinetrigger_param(['pipe'])
        self.assertEqual(self.jj.params['PIPELINE_TRIGGER'], '<au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger> <downstreamProjectNames>test-user__pipe_starter</downstreamProjectNames> </au.com.centrumsystems.hudson.plugin.buildpipeline.trigger.BuildPipelineTrigger>')

    def test__set_pipelinetrigger_param__input_empty_job_type_list__return_empty_string(self):
        self.assertEqual(self.jj.set_pipelinetrigger_param([]), '')

    # Testing set_groovypostbuild_param
    def test__set_groovypostbuild_param__input_script_type_string_project_list_behavior_string__return_groovypostbuild_config_string(self):
        self.jj.set_groovypostbuild_param('enable', self.job_type_test_list, 2)
        self.assertEqual(self.jj.params['GROOVY_POSTBUILD'], "<org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder> <groovyScript>if(manager.build.result.isBetterOrEqualTo(hudson.model.Result.FAILURE)) { manager.listener.logger.println('Because this build did not fail:' for (project in ['test-user__pipe_starter', 'test-user__prio_build', 'test-user__normal_build']) { manager.listener.logger.println(' - ' + project) manager.hudson.getItem(project).enable() } manager.listener.logger.println('will be enabled.'}</groovyScript> <behavior>2</behavior> </org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder>")

    def test__set_groovypostbuild_param__input_script_type_string_project_list_behavior_string__return_groovypostbuild_config_string2(self):
        self.jj.set_groovypostbuild_param('disable', self.job_type_test_list, 2)
        self.assertEqual(self.jj.params['GROOVY_POSTBUILD'], "<org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder> <groovyScript>for (project in ['test-user__pipe_starter', 'test-user__prio_build', 'test-user__normal_build']) { manager.listener.logger.println('Disable ' + project + ' job') manager.hudson.getItem(project).disable() }</groovyScript> <behavior>2</behavior> </org.jvnet.hudson.plugins.groovypostbuild.GroovyPostbuildRecorder>")

    def test__set_groovypostbuild_param__input_empty_script_type_string__raise_exception(self):
        self.assertRaises(Exception, self.jj.set_groovypostbuild_param, '', self.job_type_test_list, 2)

    def test__set_groovypostbuild_param__input_invalid_script_type_string__raise_exception(self):
        self.assertRaises(KeyError, self.jj.set_groovypostbuild_param, 'invalid', self.job_type_test_list, 2)

    def test__set_groovypostbuild_param__input_empty_project_list__raise_exception(self):
        self.assertRaises(Exception, self.jj.set_groovypostbuild_param, 'enable', [], 2)

    def test__set_groovypostbuild_param__input_invalid_project_list_entry__raise_exception(self):
        self.assertRaises(Exception, self.jj.set_groovypostbuild_param, 'enable', ['invalid'], 2)

    def test__set_groovypostbuild_param__input_invalid_behavior__raise_exception(self):
        self.assertRaises(Exception, self.jj.set_groovypostbuild_param, 'enable', self.job_type_test_list, 3)

    def test__set_groovypostbuild_param__input_invalid_behavior__raise_exception2(self):
        self.assertRaises(Exception, self.jj.set_groovypostbuild_param, 'enable', self.job_type_test_list, -1)

    def test__set_groovypostbuild_param__input_invalid_behavior__raise_exception3(self):
        self.assertRaises(Exception, self.jj.set_groovypostbuild_param, 'enable', self.job_type_test_list, '1')

    # Testing get_single_parameterizedtrigger
    def test__get_single_parameterizedtrigger__input_job_type_list__check_set_param(self):
        result = self.jj.get_single_parameterizedtrigger(self.job_type_test_list)
        self.assertEqual(result, '<hudson.plugins.parameterizedtrigger.BuildTriggerConfig> <configs>  </configs> <projects> test-user__pipe_starter, test-user__prio_build, test-user__normal_build </projects> <condition>SUCCESS</condition> <triggerWithNoParameters>false</triggerWithNoParameters> </hudson.plugins.parameterizedtrigger.BuildTriggerConfig>')

    def test__get_single_parameterizedtrigger__input_job_type_list_and_subset_filter_str__check_set_param(self):
        result = self.jj.get_single_parameterizedtrigger(self.job_type_test_list, subset_filter='test==filter')
        self.assertEqual(result, '<hudson.plugins.parameterizedtrigger.BuildTriggerConfig> <configs> <hudson.plugins.parameterizedtrigger.matrix.MatrixSubsetBuildParameters> <filter>test==filter</filter> </hudson.plugins.parameterizedtrigger.matrix.MatrixSubsetBuildParameters> </configs> <projects> test-user__pipe_starter, test-user__prio_build, test-user__normal_build </projects> <condition>SUCCESS</condition> <triggerWithNoParameters>false</triggerWithNoParameters> </hudson.plugins.parameterizedtrigger.BuildTriggerConfig>')

    def test__get_single_parameterizedtrigger__input_job_type_list_and_subset_filter_str__check_set_param2(self):
        result = self.jj.get_single_parameterizedtrigger(self.job_type_test_list, predefined_param='PARAM=value')
        self.assertEqual(result, '<hudson.plugins.parameterizedtrigger.BuildTriggerConfig> <configs> <hudson.plugins.parameterizedtrigger.PredefinedBuildParameters> <properties>PARAM=value</properties> </hudson.plugins.parameterizedtrigger.PredefinedBuildParameters> </configs> <projects> test-user__pipe_starter, test-user__prio_build, test-user__normal_build </projects> <condition>SUCCESS</condition> <triggerWithNoParameters>false</triggerWithNoParameters> </hudson.plugins.parameterizedtrigger.BuildTriggerConfig>')

    def test__get_single_parameterizedtrigger__input_job_type_list_and_condition_str__check_set_param(self):
        result = self.jj.get_single_parameterizedtrigger(self.job_type_test_list, condition='FAILURE')
        self.assertEqual(result, '<hudson.plugins.parameterizedtrigger.BuildTriggerConfig> <configs>  </configs> <projects> test-user__pipe_starter, test-user__prio_build, test-user__normal_build </projects> <condition>FAILURE</condition> <triggerWithNoParameters>false</triggerWithNoParameters> </hudson.plugins.parameterizedtrigger.BuildTriggerConfig>')

    def test__get_single_parameterizedtrigger__input_job_type_list_and_no_param_bool__check_set_param(self):
        result = self.jj.get_single_parameterizedtrigger(self.job_type_test_list, no_param=True)
        self.assertEqual(result, '<hudson.plugins.parameterizedtrigger.BuildTriggerConfig> <configs>  </configs> <projects> test-user__pipe_starter, test-user__prio_build, test-user__normal_build </projects> <condition>SUCCESS</condition> <triggerWithNoParameters>true</triggerWithNoParameters> </hudson.plugins.parameterizedtrigger.BuildTriggerConfig>')

    # Testing set_parameterizedtrigger_param
    def test__set_parameterizedtrigger_param__input_trigger_config_str_list__check_set_param(self):
        trigger_config = '<hudson.plugins.parameterizedtrigger.BuildTriggerConfig> <configs>  </configs> <projects> test-user__pipe_starter, test-user__prio_build, test-user__normal_build </projects> <condition>SUCCESS</condition> <triggerWithNoParameters>false</triggerWithNoParameters> </hudson.plugins.parameterizedtrigger.BuildTriggerConfig>'
        self.jj.set_parameterizedtrigger_param([trigger_config])
        self.assertEqual(self.jj.params['PARAMETERIZED_TRIGGER'], '<hudson.plugins.parameterizedtrigger.BuildTrigger> <configs> <hudson.plugins.parameterizedtrigger.BuildTriggerConfig> <configs>  </configs> <projects> test-user__pipe_starter, test-user__prio_build, test-user__normal_build </projects> <condition>SUCCESS</condition> <triggerWithNoParameters>false</triggerWithNoParameters> </hudson.plugins.parameterizedtrigger.BuildTriggerConfig> </configs> </hudson.plugins.parameterizedtrigger.BuildTrigger>')

    def test__set_parameterizedtrigger_param__input_trigger_config_str_list__check_set_param2(self):
        trigger_config1 = '<hudson.plugins.parameterizedtrigger.BuildTriggerConfig> <configs>  </configs> <projects> test-user__pipe_starter, test-user__prio_build, test-user__normal_build </projects> <condition>SUCCESS</condition> <triggerWithNoParameters>false</triggerWithNoParameters> </hudson.plugins.parameterizedtrigger.BuildTriggerConfig>'
        trigger_config2 = '<hudson.plugins.parameterizedtrigger.BuildTriggerConfig> <configs>  </configs> <projects> test-user__normal_build2 </projects> <condition>SUCCESS</condition> <triggerWithNoParameters>false</triggerWithNoParameters> </hudson.plugins.parameterizedtrigger.BuildTriggerConfig>'
        self.jj.set_parameterizedtrigger_param([trigger_config1, trigger_config2])
        self.assertEqual(self.jj.params['PARAMETERIZED_TRIGGER'], '<hudson.plugins.parameterizedtrigger.BuildTrigger> <configs> <hudson.plugins.parameterizedtrigger.BuildTriggerConfig> <configs>  </configs> <projects> test-user__pipe_starter, test-user__prio_build, test-user__normal_build </projects> <condition>SUCCESS</condition> <triggerWithNoParameters>false</triggerWithNoParameters> </hudson.plugins.parameterizedtrigger.BuildTriggerConfig> <hudson.plugins.parameterizedtrigger.BuildTriggerConfig> <configs>  </configs> <projects> test-user__normal_build2 </projects> <condition>SUCCESS</condition> <triggerWithNoParameters>false</triggerWithNoParameters> </hudson.plugins.parameterizedtrigger.BuildTriggerConfig> </configs> </hudson.plugins.parameterizedtrigger.BuildTrigger>')

    def test__set_parameterizedtrigger_param__input_empty_list__raise_exception(self):
        self.assertRaises(Exception, self.jj.set_parameterizedtrigger_param, [])

    # Testing set_mailer_param
    def test__set_mailer_param__check_set_param(self):
        self.jj.set_mailer_param()
        self.assertEqual(self.jj.params['MAILER'], '<hudson.tasks.Mailer> <recipients>test@ipa.fhg.de</recipients> <dontNotifyEveryUnstableBuild>false</dontNotifyEveryUnstableBuild> <sendToIndividuals>false</sendToIndividuals> </hudson.tasks.Mailer>')

    def test__set_junit_testresults__check_set_param(self):
        self.jj.set_junit_testresults_param()
        self.assertEqual(self.jj.params['JUNIT_TESTRESULTS'], '<hudson.tasks.junit.JUnitResultArchiver> <testResults>test_results/*.xml</testResults> <keepLongStdio>false</keepLongStdio> <testDataPublishers/> </hudson.tasks.junit.JUnitResultArchiver>')

    def test__set_trigger_param_input_vcs__check_set_param(self):
        self.jj.params['VCS'] = ''
        self.jj.repo_list = ['test_repo_2']
        self.jj.poll = self.jj.repo_list[0]
        self.jj.set_trigger_param('vcs')
        self.assertEqual(self.jj.params['TRIGGER'], '<triggers class="vector"> <hudson.triggers.SCMTrigger> <spec>*/10 * * * *</spec> </hudson.triggers.SCMTrigger> </triggers>')
        self.assertTrue(self.jj.params['VCS'] != '')

    def test__set_trigger_param_input_resulttrigger__check_set_param(self):
        # TODO
        pass

    def test__set_vcs_param__check_git(self):
        self.jj.repo_list = ['test_repo_2']
        self.jj.poll = self.jj.repo_list[0]
        self.jj.set_vcs_param()
        self.assertEqual(self.jj.params['VCS'], '<scm class="hudson.plugins.git.GitSCM"> <configVersion>2</configVersion> <userRemoteConfigs> <hudson.plugins.git.UserRemoteConfig> <name>origin</name> <refspec>+refs/heads/test:refs/remotes/origin/test</refspec> <url>git@github.com/user/repo2</url> </hudson.plugins.git.UserRemoteConfig> </userRemoteConfigs> <branches> <hudson.plugins.git.BranchSpec> <name>test</name> </hudson.plugins.git.BranchSpec> </branches> <disableSubmodules>false</disableSubmodules> <recursiveSubmodules>true</recursiveSubmodules> <doGenerateSubmoduleConfigurations>false</doGenerateSubmoduleConfigurations> <authorOrCommitter>false</authorOrCommitter> <clean>false</clean> <wipeOutWorkspace>false</wipeOutWorkspace> <pruneBranches>false</pruneBranches> <remotePoll>false</remotePoll> <ignoreNotifyCommit>false</ignoreNotifyCommit> <buildChooser class="hudson.plugins.git.util.DefaultBuildChooser"/> <gitTool>Default</gitTool> <submoduleCfg class="list"/> <relativeTargetDir>monitored_vcs</relativeTargetDir> <reference/> <excludedRegions/> <excludedUsers/> <gitConfigName/> <gitConfigEmail/> <skipTag>false</skipTag> <includedRegions/> <scmName/> </scm>')

    def test__set_vcs_param__check_git2(self):
        self.jj.poll = 'dep_repo_1'
        self.jj.repo_list = ['test_repo_2']
        self.jj.set_vcs_param()
        self.assertEqual(self.jj.params['VCS'], '<scm class="hudson.plugins.git.GitSCM"> <configVersion>2</configVersion> <userRemoteConfigs> <hudson.plugins.git.UserRemoteConfig> <name>origin</name> <refspec>+refs/heads/master:refs/remotes/origin/master</refspec> <url>git@github.com/user_x/dep_repo_1</url> </hudson.plugins.git.UserRemoteConfig> </userRemoteConfigs> <branches> <hudson.plugins.git.BranchSpec> <name>master</name> </hudson.plugins.git.BranchSpec> </branches> <disableSubmodules>false</disableSubmodules> <recursiveSubmodules>true</recursiveSubmodules> <doGenerateSubmoduleConfigurations>false</doGenerateSubmoduleConfigurations> <authorOrCommitter>false</authorOrCommitter> <clean>false</clean> <wipeOutWorkspace>false</wipeOutWorkspace> <pruneBranches>false</pruneBranches> <remotePoll>false</remotePoll> <ignoreNotifyCommit>false</ignoreNotifyCommit> <buildChooser class="hudson.plugins.git.util.DefaultBuildChooser"/> <gitTool>Default</gitTool> <submoduleCfg class="list"/> <relativeTargetDir>monitored_vcs</relativeTargetDir> <reference/> <excludedRegions/> <excludedUsers/> <gitConfigName/> <gitConfigEmail/> <skipTag>false</skipTag> <includedRegions/> <scmName/> </scm>')

    def test__set_vcs_param__check_hg(self):
        self.jj.repo_list = ['test_repo_3']
        self.jj.poll = self.jj.repo_list[0]
        self.jj.set_vcs_param()
        self.assertEqual(self.jj.params['VCS'], '<scm class="hudson.plugins.mercurial.MercurialSCM"> <source>https://kforge.ros.org/test/test_repo_3</source> <modules/> <subdir>monitored_vcs</subdir> <clean>false</clean> <forest>false</forest> <branch>master</branch> </scm>')

    def test__set_vcs_param__check_hg2(self):
        self.jj.poll = 'dep_repo_4'
        self.jj.repo_list = ['test_repo_2']
        self.jj.set_vcs_param()
        self.assertEqual(self.jj.params['VCS'], '<scm class="hudson.plugins.mercurial.MercurialSCM"> <source>https://kforge.ros.org/test/dep_repo_4</source> <modules/> <subdir>monitored_vcs</subdir> <clean>false</clean> <forest>false</forest> <branch>master</branch> </scm>')

    def test__set_vcs_param__check_svn(self):
        self.jj.repo_list = ['test_repo_4']
        self.jj.poll = self.jj.repo_list[0]
        self.jj.set_vcs_param()
        self.assertEqual(self.jj.params['VCS'], '<scm class="hudson.scm.SubversionSCM"> <locations> <hudson.scm.SubversionSCM_-ModuleLocation> <remote>https://code.test.org/svn/test/stacks/test_repo_4</remote> <local>monitored_vcs</local> </hudson.scm.SubversionSCM_-ModuleLocation> </locations> <useUpdate>false</useUpdate> <doRevert>false</doRevert> <excludedRegions/> <includedRegions/> <excludedUsers/> <excludedRevprop/> <excludedCommitMessages/> </scm>')

    def test__set_vcs_param__check_svn2(self):
        self.jj.poll = 'dep_repo_5'
        self.jj.repo_list = ['test_repo_2']
        self.jj.set_vcs_param()
        self.assertEqual(self.jj.params['VCS'], '<scm class="hudson.scm.SubversionSCM"> <locations> <hudson.scm.SubversionSCM_-ModuleLocation> <remote>https://code.test.org/svn/test/stacks/dep_repo_5</remote> <local>monitored_vcs</local> </hudson.scm.SubversionSCM_-ModuleLocation> </locations> <useUpdate>false</useUpdate> <doRevert>false</doRevert> <excludedRegions/> <includedRegions/> <excludedUsers/> <excludedRevprop/> <excludedCommitMessages/> </scm>')

    def test__set_shell_param__input_shell_script_str_check_param(self):
        self.jj.set_shell_param('echo test')
        self.assertEqual(self.jj.params['SHELL'], '<builders> <hudson.tasks.Shell> <command> echo test </command> </hudson.tasks.Shell> </builders>')

    def test__set_shell_param__input_empty_str_raise_exception(self):
        self.assertRaises(Exception, self.jj.set_shell_param, '')

    def test__set_shell_param__input_wrong_type_raise_exception(self):
        self.assertRaises(Exception, self.jj.set_shell_param, 2)

    def test__get_shell_script__result_shell_script_str(self):
        self.jj.job_type = 'prio'
        result = self.jj.get_shell_script()
        self.assertEqual(result, '#!/bin/bash -e\\n\nnew_basetgz=${ubuntu_distro}__${arch}__${ros_distro}\\n basetgz=${new_basetgz}__${repository}\\n\nsudo rm -rf $WORKSPACE/*\\n\nmkdir $WORKSPACE/../aux\\n\nscp jenkins@jenkins-test-server:~/chroot_tarballs/$new_basetgz $WORKSPACE/../aux/test-user__${basetgz}\\n scp jenkins@jenkins-test-server:~/jenkins-config/.gitconfig $WORKSPACE/.gitconfig\\n scp -r jenkins@jenkins-test-server:~/jenkins-config/.ssh $WORKSPACE/.ssh\\n ls -lah $WORKSPACE\\n\ngit clone git://github.com/fmw-jk/jenkins_setup.git $WORKSPACE/jenkins_setup\\n ls -lah $WORKSPACE\ncat &gt; $WORKSPACE/env_vars.sh &lt;&lt;DELIM\\n JOBNAME=$JOB_NAME\\n ROSDISTRO=$ros_distro\\n REPOSITORY=$repository\\n UBUNTUDISTRO=$ubuntu_distro\\n ARCH=$arch\\n #TODO\\n JENKINS_MASTER=jenkins-test-server\\n JENKINS_USER=test-user\\n JOBTYPE=prio\\n export ROS_TEST_RESULTS_DIR=/tmp/test_repositories/src_repository/test_results # TODO\\n DELIM\\n\nls -lah $WORKSPACE\\n\necho "***********ENTER CHROOT************"\\n echo "*********please be patient*********"\\n sudo pbuilder execute --basetgz $WORKSPACE/../aux/test-user__${basetgz} --save-after-exec --bindmounts $WORKSPACE -- $WORKSPACE/jenkins_setup/scripts/pbuilder_env.sh $WORKSPACE\\n\necho "*******CLEANUP WORKSPACE*******"\\n echo "STORING CHROOT TARBALL ON SERVER"\\n scp $WORKSPACE/../aux/test-user__${basetgz} jenkins@jenkins-test-server:~/chroot_tarballs/in_use/\\n sudo rm -rf $WORKSPACE/../aux\\n')

    def test__get_shell_script__result_shell_script_str2(self):
        self.jj.job_type = 'normal'
        result = self.jj.get_shell_script()
        self.assertEqual(result, '#!/bin/bash -e\\n\nnew_basetgz=${ubuntu_distro}__${arch}__${ros_distro}\\n basetgz=${new_basetgz}__${repository}\\n\nsudo rm -rf $WORKSPACE/*\\n\nmkdir $WORKSPACE/../aux\\n\nscp jenkins@jenkins-test-server:~/chroot_tarballs/$new_basetgz $WORKSPACE/../aux/test-user__${basetgz}\\n scp jenkins@jenkins-test-server:~/jenkins-config/.gitconfig $WORKSPACE/.gitconfig\\n scp -r jenkins@jenkins-test-server:~/jenkins-config/.ssh $WORKSPACE/.ssh\\n ls -lah $WORKSPACE\\n\ngit clone git://github.com/fmw-jk/jenkins_setup.git $WORKSPACE/jenkins_setup\\n ls -lah $WORKSPACE\\n\ncat &gt; $WORKSPACE/env_vars.sh &lt;&lt;DELIM\\n JOBNAME=$JOB_NAME\\n ROSDISTRO=$ros_distro\\n REPOSITORY=$repository\\n UBUNTUDISTRO=$ubuntu_distro\\n ARCH=$arch\\n #TODO\\n JENKINS_MASTER=jenkins-test-server\\n JENKINS_USER=test-user\\n JOBTYPE=normal\\n export ROS_TEST_RESULTS_DIR=/tmp/test_repositories/src_repository/test_results # TODO\\n DELIM\\n\nls -lah $WORKSPACE\\n\necho "***********ENTER CHROOT************"\\n echo "*********please be patient*********"\\n sudo pbuilder execute --basetgz $WORKSPACE/../aux/test-user__${basetgz} --save-after-exec --bindmounts $WORKSPACE -- $WORKSPACE/jenkins_setup/scripts/pbuilder_env.sh $WORKSPACE\\n\necho "*******CLEANUP WORKSPACE*******"\\n sudo rm -rf $WORKSPACE/../aux\\n')


class Pipe_Starter_Job_Test(unittest.TestCase):
    """
    Tests Pipe Starter jobs
    """

    def setUp(self):
        self.maxDiff = None

        #self.test_dict = cob_common.get_buildpipeline_configs('jenkins-test-server', 'test-user')
        self.test_pipe_inst = cob_pipe.Cob_Pipe()
        self.test_pipe_inst.load_config_from_url('jenkins-test-server', 'test-user')

        self.job_type_test_list = ['pipe', 'prio', 'normal']

        with open(os.path.expanduser('~/jenkins-config/slave_config.yaml')) as f:
            info = yaml.load(f)
        self.jenkins_instance = jenkins.Jenkins(info['master_url'], info['jenkins_login'], info['jenkins_pw'])

    def test__get_prio_subset_filter__result_filter_input_dict(self):
        self.jj = jenkins_job_creator.Pipe_Starter_Job(self.jenkins_instance, self.test_pipe_inst, ['test_repo_1'], 'dep_repo_1')
        result = self.jj.get_prio_subset_filter()
        self.assertEqual(result, [{'repository': 'test_repo_1', 'ros_distro': 'test_rosdistro', 'ubuntu_distro': 'oneiric', 'arch': 'amd64'},
                                  {'repository': 'test_repo_1', 'ros_distro': 'test_rosdistro_2', 'ubuntu_distro': 'oneiric', 'arch': 'amd64'}])

    def test__get_prio_subset_filter__result_filter_input_dict2(self):
        self.jj = jenkins_job_creator.Pipe_Starter_Job(self.jenkins_instance, self.test_pipe_inst, ['test_repo_1', 'test_repo_2'], 'dep_repo_1')
        result = self.jj.get_prio_subset_filter()
        self.assertEqual(result, [{'repository': 'test_repo_1', 'ros_distro': 'test_rosdistro', 'ubuntu_distro': 'oneiric', 'arch': 'amd64'},
                                  {'repository': 'test_repo_1', 'ros_distro': 'test_rosdistro_2', 'ubuntu_distro': 'oneiric', 'arch': 'amd64'},
                                  {'repository': 'test_repo_2', 'ros_distro': 'test_rosdistro', 'ubuntu_distro': 'natty', 'arch': 'amd64'}])


class Priority_Build_Job_Test(unittest.TestCase):
    """
    Tests Priority Build Job
    """

    def setUp(self):
        self.maxDiff = None

        #self.test_dict = cob_common.get_buildpipeline_configs('jenkins-test-server', 'test-user')
        self.test_pipe_inst = cob_pipe.Cob_Pipe()
        self.test_pipe_inst.load_config_from_url('jenkins-test-server', 'test-user')

        self.job_type_test_list = ['pipe', 'prio', 'normal']

        with open(os.path.expanduser('~/jenkins-config/slave_config.yaml')) as f:
            info = yaml.load(f)
        self.jenkins_instance = jenkins.Jenkins(info['master_url'], info['jenkins_login'], info['jenkins_pw'])

        self.jj = jenkins_job_creator.Priority_Build_Job(self.jenkins_instance, self.test_pipe_inst)

    def test__get_normal_subset_filter__result_filter_input_dict(self):
        result = self.jj.get_normal_subset_filter()
        self.assertEqual(result, [{'repository': 'test_repo_1', 'ros_distro': 'test_rosdistro', 'ubuntu_distro': 'lucid', 'arch': 'amd64'},
                                  {'repository': 'test_repo_1', 'ros_distro': 'test_rosdistro', 'ubuntu_distro': 'lucid', 'arch': 'i386'},
                                  {'repository': 'test_repo_1', 'ros_distro': 'test_rosdistro', 'ubuntu_distro': 'oneiric', 'arch': 'i386'},
                                  {'repository': 'test_repo_1', 'ros_distro': 'test_rosdistro_2', 'ubuntu_distro': 'lucid', 'arch': 'amd64'},
                                  {'repository': 'test_repo_1', 'ros_distro': 'test_rosdistro_2', 'ubuntu_distro': 'lucid', 'arch': 'i386'},
                                  {'repository': 'test_repo_1', 'ros_distro': 'test_rosdistro_2', 'ubuntu_distro': 'oneiric', 'arch': 'i386'},
                                  {'repository': 'test_repo_3', 'ros_distro': 'test_rosdistro', 'ubuntu_distro': 'lucid', 'arch': 'i386'}])
