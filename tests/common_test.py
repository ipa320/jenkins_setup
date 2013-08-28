#!/usr/bin/env python

import unittest
import os
import sys
from mock import MagicMock
from jenkins_setup import common


class Common_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

    def test__append_pymodules_if_needed__check_path(self):
        if os.path.abspath("/usr/lib/pymodules/python2.7") in sys.path:
            sys.path.remove("/usr/lib/pymodules/python2.7")
        common.append_pymodules_if_needed()
        self.assertTrue(os.path.abspath("/usr/lib/pymodules/python2.7") in sys.path)

    def test__apt_get_update__check_correct_call_calling(self):
        real = common
        real.call = MagicMock()
        real.apt_get_update()
        real.call.assert_called_once_with("apt-get update")

    def test__apt_get_update__input_sudo_true__check_correct_call_calling(self):
        real = common
        real.call = MagicMock()
        real.apt_get_update(True)
        real.call.assert_called_once_with("sudo apt-get update")

    def test__apt_get_install__input_list__check_correct_call_calling(self):
        real = common
        real.call = MagicMock()
        real.apt_get_install(['test-package'])
        real.call.assert_called_once_with("apt-get install --yes test-package")

    def test__apt_get_install__input_with_sudo__check_correct_call_calling(self):
        real = common
        real.call = MagicMock()
        real.apt_get_install(['test-package'], sudo=True)
        real.call.assert_called_once_with("sudo apt-get install --yes test-package")

    def test__apt_get_install__input_empty_list__check_correct_call_calling(self):
        real = common
        real.call = MagicMock()
        real.apt_get_install([])
        self.assertEqual(real.call.call_count, 0)

    def test__apt_get_install__input_with_rosdep__check_(self):
        real_common = common
        real_common.call = MagicMock()
        mock_rosdep = MagicMock()
        real_common.apt_get_install(['test-package'], rosdep=mock_rosdep)
        mock_rosdep.to_aptlist.assert_called_once_with(['test-package'])

    def test__call__input__check_call_with_list_call(self):
        real_common = common
        real_common.call_with_list = MagicMock()
        real_common.call("test command")
        real_common.call_with_list.assert_called_once_with(['test', 'command'], None, True)

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
