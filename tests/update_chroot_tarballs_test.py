import unittest

from update_chroot_tarballs import *


class Update_Chroot_Tarballs_Test(unittest.TestCase):

    def setUp(self):
        self.maxDiff = None

    def test__get_tarball_lists__return_two_list(self):
        basic_tarball_test_list = ['lucid__i386',
                                   'lucid__amd64',
                                   'maverick__i386',
                                   'maverick__amd64',
                                   'natty__i386',
                                   'natty__amd64',
                                   'oneiric__i386',
                                   'oneiric__amd64',
                                   'precise__i386',
                                   'precise__amd64',
                                   'quantal__i386',
                                   'quantal__amd64']
        extended_tarball_test_list = ['lucid__i386__electric',
                                      'lucid__i386__fuerte',
                                      'lucid__amd64__electric',
                                      'lucid__amd64__fuerte',
                                      'maverick__i386__electric',
                                      'maverick__amd64__electric',
                                      'natty__i386__electric',
                                      'natty__amd64__electric',
                                      'oneiric__i386__electric',
                                      'oneiric__i386__fuerte',
                                      'oneiric__i386__groovy',
                                      'oneiric__amd64__electric',
                                      'oneiric__amd64__fuerte',
                                      'oneiric__amd64__groovy',
                                      'precise__i386__fuerte',
                                      'precise__i386__groovy',
                                      'precise__amd64__fuerte',
                                      'precise__amd64__groovy',
                                      'quantal__i386__groovy',
                                      'quantal__amd64__groovy',
                                      ]
        result = get_tarball_lists()
        self.assertEqual(sorted(result), [sorted(basic_tarball_test_list),
                                          sorted(extended_tarball_test_list)])

    def test__get_tarball_params__input_tarball_name_string__return_params_dict(self):
        result = get_tarball_params('precise__amd64__fuerte')
        self.assertEqual(result, {'ubuntu_distro': 'precise',
                                  'arch': 'amd64',
                                  'ros_distro': 'fuerte'})

    def test__get_tarball_params__input_tarball_name_string__return_params_dict2(self):
        result = get_tarball_params('natty__i386')
        self.assertEqual(result, {'ubuntu_distro': 'natty',
                                  'arch': 'i386',
                                  'ros_distro': None})


if __name__ == '__main__':
    unittest.main()
