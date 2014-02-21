#!/usr/bin/env python

import unittest
from jenkins_setup import cob_pipe


class CobPipeTest(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

        self.cp = cob_pipe.CobPipe()
        self.cp.load_config_from_url('fmw-jk', 'jenkins-test-server', 'test-user')

        self.repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                              'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                              'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                              'regular_matrix': None, 'dependencies': None,
                                              'jobs': None, 'robots': None}}
        self.pipe_config_test_dict = {'user_name': 'test-user', 'server_name': 'test-server',
                                      'email': 'test@ipa.fhg.de', 'committer_email_enabled': False,
                                      'repositories': self.repo_test_dict}

    def test__load_config_from_url__check_user_name(self):
        self.assertEqual(self.cp.user_name, 'test-user')

    def test__load_config_from_url__check_server_name(self):
        self.assertEqual(self.cp.server_name, 'jenkins-test-server')

    def test__load_config_from_url__check_email(self):
        self.assertEqual(self.cp.email, 'test@ipa.fhg.de')

    def test__load_config_from_url__check_committer_email_enabled(self):
        self.assertEqual(self.cp.committer_email_enabled, False)

    def test__get_jobs_to_create__result_job_list(self):
        self.assertEqual(self.cp.get_jobs_to_create(), {'regular_build': ['test_repo_1', 'test_repo_3'], 'nongraphics_test': ['test_repo_1'], 'hardware_build': ['test_repo_1'], 'automatic_hw_test': ['test_repo_1'], 'release': ['test_repo_1'], 'graphics_test': ['test_repo_1'], 'interactive_hw_test': ['test_repo_1']})

    def test__load_config_from_dict__input_repo_dict__check_repo(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'regular_matrix': None, 'dependencies': None,
                                         'jobs': None, 'robots': None}}
        self.pipe_config_test_dict['repositories'] = repo_test_dict
        self.cp.load_config_from_dict(self.pipe_config_test_dict)
        self.assertTrue('cob_extern' in self.cp.repositories)
#        self.assertIn('cob_extern', cd.repositories)

    def test__load_config_from_dict__input_repo_dict__check_repo_url(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'regular_matrix': None, 'dependencies': None,
                                         'jobs': None, 'robots': None}}
        self.pipe_config_test_dict['repositories'] = repo_test_dict
        self.cp.load_config_from_dict(self.pipe_config_test_dict)
        self.assertEqual(self.cp.repositories['cob_extern'].url, 'git://github.com/ipa320/cob_extern.git')

    def test__load_config_from_dict__input_repo_dict__check_repo_type(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'regular_matrix': None, 'dependencies': None,
                                         'jobs': None, 'robots': None}}
        self.pipe_config_test_dict['repositories'] = repo_test_dict
        self.cp.load_config_from_dict(self.pipe_config_test_dict)
        self.assertEqual(self.cp.repositories['cob_extern'].type, 'git')

    def test__load_config_from_dict__input_repo_dict__check_repo_version(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'regular_matrix': None, 'dependencies': None,
                                         'jobs': None, 'robots': None}}
        self.pipe_config_test_dict['repositories'] = repo_test_dict
        self.cp.load_config_from_dict(self.pipe_config_test_dict)
        self.assertEqual(self.cp.repositories['cob_extern'].version, 'master')

    def test__load_config_from_dict__input_repo_dict__raise_exception(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'regular_matrix': None, 'dependencies': None,
                                         'jobs': [], 'robots': ['test-robot']}}
        self.pipe_config_test_dict['repositories'] = repo_test_dict
        self.assertRaises(cob_pipe.CobPipeException, self.cp.load_config_from_dict, self.pipe_config_test_dict)

    def test__load_config_from_dict__input_repo_dict__raise_exception2(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'regular_matrix': None, 'dependencies': None,
                                         'jobs': ['automatic_hw_test'], 'robots': None}}
        self.pipe_config_test_dict['repositories'] = repo_test_dict
        self.assertRaises(cob_pipe.CobPipeException, self.cp.load_config_from_dict, self.pipe_config_test_dict)

    def test__load_config_from_dict__input_repo_dict__raise_exception3(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'regular_matrix': None, 'dependencies': None,
                                         'jobs': ['interactive_hw_test'], 'robots': None}}
        self.pipe_config_test_dict['repositories'] = repo_test_dict
        self.assertRaises(cob_pipe.CobPipeException, self.cp.load_config_from_dict, self.pipe_config_test_dict)

    def test__load_config_from_url__input_repo_dict__check_repo(self):
        self.cp.load_config_from_url('fmw-jk', 'jenkins-test-server', 'test-user')
        self.assertTrue('test_repo_1' in self.cp.repositories)

    def test__get_custom_dependencies__return_dependency_dict(self):
        dep_test_dict = {'dep_1': {'type': 'git', 'url': 'git://github.com/ipa320/dep_1.git',
                                   'poll': True},
                         'dep_2': {'type': 'git', 'url': 'git://github.com/ipa320/dep_2.git',
                                   'poll': False}}
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'regular_matrix': None, 'dependencies': dep_test_dict,
                                         'jobs': None, 'robots': None}}
        self.pipe_config_test_dict['repositories'] = repo_test_dict
        self.cp = cob_pipe.CobPipe()
        self.cp.load_config_from_dict(self.pipe_config_test_dict)
        result = self.cp.get_custom_dependencies()
        self.assertEqual(result, {'dep_1': ['cob_extern'], 'dep_2': ['cob_extern']})

    def test__get_custom_dependencies__return_dependency_dict2(self):
        dep_test_dict = {'dep_1': {'type': 'git', 'url': 'git://github.com/ipa320/dep_1.git',
                                   'poll': True},
                         'dep_2': {'type': 'git', 'url': 'git://github.com/ipa320/dep_2.git',
                                   'poll': False}}
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'regular_matrix': None, 'dependencies': dep_test_dict,
                                         'jobs': None, 'robots': None},
                          'cob_driver': {'type': 'git', 'url': 'git://github.com/ipa320/cob_driver.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'regular_matrix': None, 'dependencies': dep_test_dict,
                                         'jobs': None, 'robots': None}}
        self.pipe_config_test_dict['repositories'] = repo_test_dict
        self.cp = cob_pipe.CobPipe()
        self.cp.load_config_from_dict(self.pipe_config_test_dict)
        result = self.cp.get_custom_dependencies()
        self.assertEqual(result, {'dep_1': ['cob_extern', 'cob_driver'], 'dep_2': ['cob_extern', 'cob_driver']})

    def test__get_custom_dependencies__input_polled_only_True__return_only_polled_dependencies_dict(self):
        dep_test_dict = {'dep_1': {'type': 'git', 'url': 'git://github.com/ipa320/dep_1.git',
                                   'poll': True},
                         'dep_2': {'type': 'git', 'url': 'git://github.com/ipa320/dep_2.git',
                                   'poll': False}}
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'regular_matrix': None, 'dependencies': dep_test_dict,
                                         'jobs': None, 'robots': None},
                          'cob_driver': {'type': 'git', 'url': 'git://github.com/ipa320/cob_driver.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'regular_matrix': None, 'dependencies': dep_test_dict,
                                         'jobs': None, 'robots': None}}
        self.pipe_config_test_dict['repositories'] = repo_test_dict
        self.cp = cob_pipe.CobPipe()
        self.cp.load_config_from_dict(self.pipe_config_test_dict)
        result = self.cp.get_custom_dependencies(polled_only=True)
        self.assertEqual(result, {'dep_1': ['cob_extern', 'cob_driver']})


class CobPipeDependencyRepoTest(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

    def test__init__without_name__raise_exception(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git', 'version': 'master'}

        self.assertRaises(cob_pipe.CobPipeException, cob_pipe.CobPipeDependencyRepo, '', data_test_dict)

    def test__init__without_name__raise_exception2(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git', 'version': 'master'}

        self.assertRaises(cob_pipe.CobPipeException, cob_pipe.CobPipeDependencyRepo, None, data_test_dict)

    def test__get_rosinstall__init__return_rosinstall_yaml_string(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git', 'version': 'master'}
        self.cdr = cob_pipe.CobPipeDependencyRepo('test', data_test_dict)

        result = self.cdr.get_rosinstall()
        self.assertEqual(result, "- git: {local-name: test, uri: 'git://github.com/ipa320/test.git', version: master}\n")

    def test__get_rosinstall__init_without_version__return_rosinstall_yaml_string(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git'}
        self.cdr = cob_pipe.CobPipeDependencyRepo('test', data_test_dict)

        result = self.cdr.get_rosinstall()
        self.assertEqual(result, "- git: {local-name: test, uri: 'git://github.com/ipa320/test.git'}\n")

    def test__get_rosinstall__init_name_with_suffix__return_rosinstall_yaml_string(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git', 'version': 'master'}
        self.cdr = cob_pipe.CobPipeDependencyRepo('test__suffix', data_test_dict)

        result = self.cdr.get_rosinstall()
        self.assertEqual(result, "- git: {local-name: test, uri: 'git://github.com/ipa320/test.git', version: master}\n")


class CobPipeRepoTest(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

    def test__init__input_name_str_pipeconf_dict__check_required_poll(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'regular_matrix': None, 'dependencies': None,
                          'jobs': None, 'robots': None}
        cpr = cob_pipe.CobPipeRepo('test', data_test_dict)
        self.assertEqual(cpr.poll, True)

    def test__init__input_name_str_pipeconf_dict__check_required_ros_distro(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'regular_matrix': None, 'dependencies': None,
                          'jobs': None, 'robots': None}
        cpr = cob_pipe.CobPipeRepo('test', data_test_dict)
        self.assertEqual(cpr.ros_distro, ['groovy'])

    def test__init__input_name_str_pipeconf_dict__check_required_prio_ubuntu_distro(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'regular_matrix': None, 'dependencies': None,
                          'jobs': None, 'robots': None}
        cpr = cob_pipe.CobPipeRepo('test', data_test_dict)
        self.assertEqual(cpr.prio_ubuntu_distro, 'oneiric')

    def test__init__input_name_str_pipeconf_dict__check_required_prio_arch(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'regular_matrix': None, 'dependencies': None,
                          'jobs': None, 'robots': None}
        cpr = cob_pipe.CobPipeRepo('test', data_test_dict)
        self.assertEqual(cpr.prio_arch, 'amd64')

    def test__init__input_name_str_pipeconf_dict__check_additional_regular_matrix(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'regular_matrix': {'oneiric': ['i386'], 'lucid': ['amd64', 'i386']},
                          'dependencies': None, 'jobs': ['regular_build'], 'robots': None}
        cpr = cob_pipe.CobPipeRepo('test', data_test_dict)
        self.assertEqual(cpr.regular_matrix,
                         {'oneiric': ['i386'], 'lucid': ['amd64', 'i386']})

    def test__init__input_name_str_pipeconf_dict__raise_exception(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'regular_matrix': {'oneiric': ['i386'], 'lucid': ['amd64', 'i386']},
                          'dependencies': None, 'jobs': None, 'robots': None}
        self.assertRaises(cob_pipe.CobPipeException, cob_pipe.CobPipeRepo, 'test', data_test_dict)

    def test__init__input_name_str_pipeconf_dict__check_additional_jobs(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'regular_matrix': None, 'dependencies': None,
                          'jobs': ['job_1', 'job_2'], 'robots': None}
        cpr = cob_pipe.CobPipeRepo('test', data_test_dict)
        self.assertEqual(cpr.jobs, ['job_1', 'job_2'])

    def test__init__input_name_str_pipeconf_dict__check_additional_dependencies(self):
        dep_test_dict = {'dep_1': {'type': 'git', 'url': 'git://github.com/ipa320/dep_1.git',
                                   'poll': True},
                         'dep_2': {'type': 'git', 'url': 'git://github.com/ipa320/dep_2.git',
                                   'poll': False}}
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'regular_matrix': None, 'jobs': None,
                          'dependencies': dep_test_dict, 'robots': None}
        cpr = cob_pipe.CobPipeRepo('test', data_test_dict)
        self.assertTrue(isinstance(cpr.dependencies['dep_1'], cob_pipe.CobPipeDependencyRepo))
#        self.assertIsInstance(cpr.dependencies['dep_1'], cob_pipe.CobPipeRepo)

    def test__init__input_name_str_pipeconf_dict__check_additional_dependencies2(self):
        dep_test_dict = {'dep_1': {'type': 'git', 'url': 'git://github.com/ipa320/dep_1.git',
                                   'poll': True},
                         'dep_2': {'type': 'git', 'url': 'git://github.com/ipa320/dep_2.git',
                                   'poll': False}}
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'regular_matrix': None, 'jobs': None,
                          'dependencies': dep_test_dict, 'robots': None}
        cpr = cob_pipe.CobPipeRepo('test', data_test_dict)
        self.assertEqual(cpr.dependencies['dep_2'].url, 'git://github.com/ipa320/dep_2.git')
