#!/usr/bin/env python

import unittest
from jenkins_setup import cob_distro


class Cob_Distro_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

    def test__init__input_rosdistro__check_url(self):
        cd = cob_distro.Cob_Distro('test')
        self.assertEqual(cd.url, 'https://raw.github.com/fmw-jk/jenkins_setup/master/releases/cob_test.yaml')

    def test__init__input_rosdistro__check_url2(self):
        cd = cob_distro.Cob_Distro('fuerte')
        self.assertEqual(cd.url, 'https://raw.github.com/fmw-jk/jenkins_setup/master/releases/cob_fuerte.yaml')


class Cob_Distro_Pipe_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

        self.cdp = cob_distro.Cob_Distro_Pipe()

    def test__load_from_dict__input_repo_dict__check_repo(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'matrix_distro_arch': None, 'dependencies': None,
                                         'jobs': None}}
        self.cdp.load_from_dict(repo_test_dict)
        self.assertTrue('cob_extern' in self.cdp.repositories)
#        self.assertIn('cob_extern', cd.repositories)

    def test__load_from_dict__input_repo_dict__check_repo_url(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'matrix_distro_arch': None, 'dependencies': None,
                                         'jobs': None}}
        self.cdp.load_from_dict(repo_test_dict)
        self.assertEqual(self.cdp.repositories['cob_extern'].url, 'git://github.com/ipa320/cob_extern.git')

    def test__load_from_dict__input_repo_dict__check_repo_type(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'matrix_distro_arch': None, 'dependencies': None,
                                         'jobs': None}}
        self.cdp.load_from_dict(repo_test_dict)
        self.assertEqual(self.cdp.repositories['cob_extern'].type, 'git')

    def test__load_from_dict__input_repo_dict__check_repo_version(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'matrix_distro_arch': None, 'dependencies': None,
                                         'jobs': None}}
        self.cdp.load_from_dict(repo_test_dict)
        self.assertEqual(self.cdp.repositories['cob_extern'].version, 'master')

    def test__load_from_url__input_repo_dict__check_repo(self):
        self.cdp.load_from_url('jenkins_test_server', 'test_user')
        self.assertTrue('test_repo_1' in self.cdp.repositories)

    def test__get_custom_dependencies__return_dependency_dict(self):
        dep_test_dict = {'dep_1': {'type': 'git', 'url': 'git://github.com/ipa320/dep_1.git',
                                   'poll': True},
                         'dep_2': {'type': 'git', 'url': 'git://github.com/ipa320/dep_2.git',
                                   'poll': False}}
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'matrix_distro_arch': None, 'dependencies': dep_test_dict,
                                         'jobs': None}}
        self.cdp.load_from_dict(repo_test_dict)
        result = self.cdp.get_custom_dependencies()
        self.assertEqual(result, {'dep_1': ['cob_extern'], 'dep_2': ['cob_extern']})

    def test__get_custom_dependencies__return_dependency_dict2(self):
        dep_test_dict = {'dep_1': {'type': 'git', 'url': 'git://github.com/ipa320/dep_1.git',
                                   'poll': True},
                         'dep_2': {'type': 'git', 'url': 'git://github.com/ipa320/dep_2.git',
                                   'poll': False}}
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'matrix_distro_arch': None, 'dependencies': dep_test_dict,
                                         'jobs': None},
                          'cob_driver': {'type': 'git', 'url': 'git://github.com/ipa320/cob_driver.git',
                                         'version': 'master', 'poll': True, 'ros_distro': ['groovy'],
                                         'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                                         'matrix_distro_arch': None, 'dependencies': dep_test_dict,
                                         'jobs': None}}
        self.cdp.load_from_dict(repo_test_dict)
        result = self.cdp.get_custom_dependencies()
        self.assertEqual(result, {'dep_1': ['cob_extern', 'cob_driver'], 'dep_2': ['cob_extern', 'cob_driver']})


class Cob_Distro_Repo_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

    def test__get_rosinstall__init__return_rosinstall_yaml_string(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git', 'version': 'master'}
        self.cdr = cob_distro.Cob_Distro_Repo('test', data_test_dict)

        result = self.cdr.get_rosinstall()
        self.assertEqual(result, "- git: {local-name: test, uri: 'git://github.com/ipa320/test.git', version: master}\n")

    def test__get_rosinstall__init_without_version__return_rosinstall_yaml_string(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git'}
        self.cdr = cob_distro.Cob_Distro_Repo('test', data_test_dict)

        result = self.cdr.get_rosinstall()
        self.assertEqual(result, "- git: {local-name: test, uri: 'git://github.com/ipa320/test.git'}\n")


class Cob_Distro_Pipe_Repo_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

    def test__init__input_name_str_pipeconf_dict__check_required_poll(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'matrix_distro_arch': None, 'dependencies': None,
                          'jobs': None}
        cdpr = cob_distro.Cob_Distro_Pipe_Repo('test', data_test_dict)
        self.assertEqual(cdpr.poll, True)

    def test__init__input_name_str_pipeconf_dict__check_required_ros_distro(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'matrix_distro_arch': None, 'dependencies': None,
                          'jobs': None}
        cdpr = cob_distro.Cob_Distro_Pipe_Repo('test', data_test_dict)
        self.assertEqual(cdpr.ros_distro, ['groovy'])

    def test__init__input_name_str_pipeconf_dict__check_required_prio_ubuntu_distro(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'matrix_distro_arch': None, 'dependencies': None,
                          'jobs': None}
        cdpr = cob_distro.Cob_Distro_Pipe_Repo('test', data_test_dict)
        self.assertEqual(cdpr.prio_ubuntu_distro, 'oneiric')

    def test__init__input_name_str_pipeconf_dict__check_required_prio_arch(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'matrix_distro_arch': None, 'dependencies': None,
                          'jobs': None}
        cdpr = cob_distro.Cob_Distro_Pipe_Repo('test', data_test_dict)
        self.assertEqual(cdpr.prio_arch, 'amd64')

    def test__init__input_name_str_pipeconf_dict__check_additional_matrix_distro_arch(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'matrix_distro_arch': {'oneiric': ['i386'], 'lucid': ['amd64', 'i386']},
                          'dependencies': None, 'jobs': None}
        cdpr = cob_distro.Cob_Distro_Pipe_Repo('test', data_test_dict)
        self.assertEqual(cdpr.matrix_distro_arch,
                         {'oneiric': ['i386'], 'lucid': ['amd64', 'i386']})

    def test__init__input_name_str_pipeconf_dict__check_additional_jobs(self):
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'matrix_distro_arch': None, 'dependencies': None,
                          'jobs': ['job_1', 'job_2']}
        cdpr = cob_distro.Cob_Distro_Pipe_Repo('test', data_test_dict)
        self.assertEqual(cdpr.jobs, ['job_1', 'job_2'])

    def test__init__input_name_str_pipeconf_dict__check_additional_dependencies(self):
        dep_test_dict = {'dep_1': {'type': 'git', 'url': 'git://github.com/ipa320/dep_1.git',
                                   'poll': True},
                         'dep_2': {'type': 'git', 'url': 'git://github.com/ipa320/dep_2.git',
                                   'poll': False}}
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'matrix_distro_arch': None, 'jobs': None,
                          'dependencies': dep_test_dict}
        cdpr = cob_distro.Cob_Distro_Pipe_Repo('test', data_test_dict)
        self.assertTrue(isinstance(cdpr.dependencies['dep_1'], cob_distro.Cob_Distro_Repo))
#        self.assertIsInstance(cdpr.dependencies['dep_1'], cob_distro.Cob_Distro_Repo)

    def test__init__input_name_str_pipeconf_dict__check_additional_dependencies2(self):
        dep_test_dict = {'dep_1': {'type': 'git', 'url': 'git://github.com/ipa320/dep_1.git',
                                   'poll': True},
                         'dep_2': {'type': 'git', 'url': 'git://github.com/ipa320/dep_2.git',
                                   'poll': False}}
        data_test_dict = {'type': 'git', 'url': 'git://github.com/ipa320/test.git',
                          'poll': True, 'ros_distro': ['groovy'],
                          'prio_ubuntu_distro': 'oneiric', 'prio_arch': 'amd64',
                          'matrix_distro_arch': None, 'jobs': None,
                          'dependencies': dep_test_dict}
        cdpr = cob_distro.Cob_Distro_Pipe_Repo('test', data_test_dict)
        self.assertEqual(cdpr.dependencies['dep_2'].url, 'git://github.com/ipa320/dep_2.git')


if __name__ == "__main__":
    unittest.main()
