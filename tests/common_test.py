#!/usr/bin/env python

import unittest
from jenkins_setup import common


class Common_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

    def test__get_all_packages__input_source_folder_str__return_package_dict(self):
        pass

    def test__get_buildpipeline_configs__input_server_and_user_name_string__return_configs_dict(self):
        result = common.get_buildpipeline_configs('git@github.com:fmw-jk/jenkins_config.git', 'jenkins-test-server', 'test-user')
        self.assertEqual(type(result), dict)

    def test__get_buildpipeline_configs__input_server_and_user_name_string__check_entries(self):
        result = common.get_buildpipeline_configs('git@github.com:fmw-jk/jenkins_config.git', 'jenkins-test-server', 'test-user')
        self.assertEqual(result['user_name'], 'test-user')

    def test__get_buildpipeline_configs__input_wrong_name__raise_exception(self):
        self.assertRaises(Exception, common.get_buildpipeline_configs, 'wrong-name', 'test-user')

    def test__get_buildpipeline_configs__input_wrong_name__raise_exception2(self):
        self.assertRaises(Exception, common.get_buildpipeline_configs, 'jenkins-test-server', 'wrong-name')


if __name__ == "__main__":
    unittest.main()
