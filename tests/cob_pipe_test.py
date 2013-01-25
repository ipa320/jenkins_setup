#!/usr/bin/env python

import unittest
from jenkins_setup import cob_pipe


class Cob_Pipe_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

        self.cp = cob_pipe.Cob_Pipe()
        self.cp.load_config_from_url('jenkins-test-server', 'test-user')

    def test__load_config_from_url__check_pipeline_github_ur(self):
        self.assertEqual(self.cp.pipeline_github_url, 'git@github.com/fmw-jk/jenkins_pipelines/jenkins-test-server/test-user')

    def test__load_config_from_url__check_user_name(self):
        self.assertEqual(self.cp.user_name, 'test-user')

    def test__load_config_from_url__check_user_group(self):
        self.assertEqual(self.cp.user_group, 'test-group')

    def test__load_config_from_url__check_server_name(self):
        self.assertEqual(self.cp.server_name, 'jenkins-test-server')

    def test__load_config_from_url__check_email(self):
        self.assertEqual(self.cp.email, 'test@ipa.fhg.de')

    def test__load_config_from_url__check_committer(self):
        self.assertEqual(self.cp.committer, False)
