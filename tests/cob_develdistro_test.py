#!/usr/bin/env python

import unittest
from jenkins_setup import cob_develdistro


class CobDevelDistroTest(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

    def test__init__input_rosdistro__check_url(self):
        cd = cob_develdistro.CobDevelDistro('test')
        self.assertEqual(cd.url, 'https://raw.github.com/ipa320/jenkins_setup/master/releases/cob_test-devel.yaml')

    def test__init__input_rosdistro__check_url2(self):
        cd = cob_develdistro.CobDevelDistro('fuerte')
        self.assertEqual(cd.url, 'https://raw.github.com/ipa320/jenkins_setup/master/releases/cob_fuerte-devel.yaml')


if __name__ == "__main__":
    unittest.main()
