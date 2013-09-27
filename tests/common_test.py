#!/usr/bin/env python2.7

import unittest
import os
import sys
from mock import MagicMock, patch
from jenkins_setup import common


class Common_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

    def test__append_pymodules_if_needed__check_path(self):
        if os.path.abspath("/usr/lib/pymodules/python2.7") in sys.path:
            sys.path.remove("/usr/lib/pymodules/python2.7")
        common.append_pymodules_if_needed()
        self.assertTrue(os.path.abspath("/usr/lib/pymodules/python2.7") in sys.path)

    @patch('jenkins_setup.common.call')
    def test__apt_get_update__check_correct_call_calling(self, mock_call):
        common.apt_get_update()
        mock_call.assert_called_once_with("apt-get update")

    @patch('jenkins_setup.common.call')
    def test__apt_get_update__input_sudo_true__check_correct_call_calling(self, mock_call):
        common.apt_get_update(True)
        mock_call.assert_called_once_with("sudo apt-get update")

    # different way of implementing the patch
    def test__apt_get_install__input_list__check_correct_call_calling(self):
        with patch('jenkins_setup.common.call') as mock_call:
            common.apt_get_install(['test-package'])
            mock_call.assert_called_once_with("apt-get install --yes test-package")

    @patch('jenkins_setup.common.call')
    def test__apt_get_install__input_with_sudo__check_correct_call_calling(self, mock_call):
        common.apt_get_install(['test-package'], sudo=True)
        mock_call.assert_called_once_with("sudo apt-get install --yes test-package")

    @patch('jenkins_setup.common.call')
    def test__apt_get_install__input_empty_list__check_correct_call_calling(self, mock_call):
        common.apt_get_install([])
        self.assertEqual(mock_call.call_count, 0)

    @patch('jenkins_setup.common.call')
    def test__apt_get_install__input_with_rosdep__check_(self, mock_call):
        mock_rosdep = MagicMock()
        common.apt_get_install(['test-package'], rosdep=mock_rosdep)
        mock_rosdep.to_aptlist.assert_called_once_with(['test-package'])

    @patch('jenkins_setup.common.call_with_list')
    def test__call__input_command_string__check_call_with_list_call(self, mock_call_with_list):
        common.call("test command")
        mock_call_with_list.assert_called_with(['test', 'command'], None, True)

    def test__get_all_packages__input_source_folder_str__return_package_dict(self):
        pass

    def test__get_buildpipeline_configs__input_server_and_user_name_string__return_configs_dict(self):
        result = common.get_buildpipeline_configs('jenkins-test-server', 'test-user', 'git@github.com:fmw-jk/jenkins_config.git')
        self.assertEqual(type(result), dict)

    def test__get_buildpipeline_configs__input_server_and_user_name_string__check_entries(self):
        result = common.get_buildpipeline_configs('jenkins-test-server', 'test-user', 'git@github.com:fmw-jk/jenkins_config.git')
        self.assertEqual(result['user_name'], 'test-user')

    def test__get_buildpipeline_configs__input_wrong_name__raise_exception(self):
        self.assertRaises(Exception, common.get_buildpipeline_configs, 'wrong-name', 'test-user')

    def test__get_buildpipeline_configs__input_wrong_name__raise_exception2(self):
        self.assertRaises(Exception, common.get_buildpipeline_configs, 'jenkins-test-server', 'wrong-name')


if __name__ == "__main__":
    unittest.main()
