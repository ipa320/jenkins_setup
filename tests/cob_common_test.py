#!/usr/bin/env python

import unittest
from jenkins_setup import cob_common


class Cob_Common_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

    def test__get_buildpipeline_configs__input_pipeline_name_string__return_configs_dict(self):
        result = cob_common.get_buildpipeline_configs('test_user__jenkins_test_server')
        self.assertEqual(type(result), dict)

    def test__get_buildpipeline_configs__input_pipeline_name_string__check_entries(self):
        result = cob_common.get_buildpipeline_configs('test_user__jenkins_test_server')
        self.assertEqual(result['user_name'], 'test_user')

    def test__get_buildpipeline_configs__input_wrong_pipeline_name__raise_exception(self):
        self.assertRaises(Exception, cob_common.get_buildpipeline_configs, 'wrong_pipeline_name')

if __name__ == "__main__":
    unittest.main()
