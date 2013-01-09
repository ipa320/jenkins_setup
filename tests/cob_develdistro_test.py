#!/usr/bin/env python

import unittest
from jenkins_setup import cob_develdistro


class Cob_Distro_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

    def test__init__input_rosdistro__check_url(self):
        cd = cob_develdistro.Cob_Distro('test')
        self.assertEqual(cd.url, 'https://raw.github.com/fmw-jk/jenkins_setup/master/releases/cob_test.yaml')

    def test__init__input_rosdistro__check_url2(self):
        cd = cob_develdistro.Cob_Distro('fuerte')
        self.assertEqual(cd.url, 'https://raw.github.com/fmw-jk/jenkins_setup/master/releases/cob_fuerte.yaml')

    def test__init__input_rosdistro_and_repo_distro__check_repo(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git', 'version': 'master'}}
        cd = cob_develdistro.Cob_Distro('test', repo_test_dict)
        self.assertIn('cob_extern', cd.repositories)

    def test__init__input_rosdistro_and_repo_distro__check_repo_url(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git', 'version': 'master'}}
        cd = cob_develdistro.Cob_Distro('test', repo_test_dict)
        self.assertEqual(cd.repositories['cob_extern'].url, 'git://github.com/ipa320/cob_extern.git')

    def test__init__input_rosdistro_and_repo_distro__check_repo_type(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git', 'version': 'master'}}
        cd = cob_develdistro.Cob_Distro('test', repo_test_dict)
        self.assertEqual(cd.repositories['cob_extern'].type, 'git')

    def test__init__input_rosdistro_and_repo_distro__check_repo_version(self):
        repo_test_dict = {'cob_extern': {'type': 'git', 'url': 'git://github.com/ipa320/cob_extern.git', 'version': 'master'}}
        cd = cob_develdistro.Cob_Distro('test', repo_test_dict)
        self.assertEqual(cd.repositories['cob_extern'].version, 'master')


class Cob_Distro_Repo_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None


if __name__ == "__main__":
    unittest.main()
