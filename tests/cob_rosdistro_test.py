#!/usr/bin/env python

import unittest
from jenkins_setup import cob_rosdistro


class Cobdistro_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

        self.cobdis = cob_rosdistro.CobDistroFile('fuerte')


if __name__ == "__main__":
    unittest.main()
