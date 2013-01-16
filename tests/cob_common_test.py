#!/usr/bin/env python

import unittest
from jenkins_setup import cob_common


class Cob_Common_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

    def test__get_dependencies_for_dry_repos__input_source_folder_str__return_dependencies_list(self):
        pass

    def test__get_buildpipeline_configs__input_server_and_user_name_string__return_configs_dict(self):
        result = cob_common.get_buildpipeline_configs('jenkins_test_server', 'test_user')
        self.assertEqual(type(result), dict)

    def test__get_buildpipeline_configs__input_server_and_user_name_string__check_entries(self):
        result = cob_common.get_buildpipeline_configs('jenkins_test_server', 'test_user')
        self.assertEqual(result['user_name'], 'test_user')

    def test__get_buildpipeline_configs__input_wrong_name__raise_exception(self):
        self.assertRaises(Exception, cob_common.get_buildpipeline_configs, 'wrong_name', 'test_user')

    def test__get_buildpipeline_configs__input_wrong_name__raise_exception2(self):
        self.assertRaises(Exception, cob_common.get_buildpipeline_configs, 'jenkins_test_server', 'wrong_name')


if __name__ == "__main__":
    unittest.main()
