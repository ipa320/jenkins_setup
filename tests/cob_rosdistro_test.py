#!/usr/bin/env python

import unittest
from jenkins_setup import cobdistro


class Cobdistro_Test(unittest.TestCase):

    def setUp(self):
        self.MaxDiff = None

        self.cobdis = cobdistro.CobDistroFile('fuerte')


if __name__ == "__main__":
    unittest.main()
