#!/usr/bin/env python

from jenkins_setup import cob_common


def apt_get_install_also_nonrosdep(pkgs, ros_distro, rosdep=None, sudo=False):
    """
    Extends cob_common.apt_get_install by trying to guess Debian package names
    of packages not included in rosdep

    @param pkgs: names of ros repositories
    @type  pkgs: list
    @param ros_distro: name of ros release, e.g. fuerte
    @type  ros_distro: str
    @param rosdep: rosdep resolver object
    @type  rosdep: rosdep.RosDepResolver
    @param sudo: execute command as super-user
    @type  sudo: bool
    """

    rosdep_pkgs = []
    aptget_pkgs = []

    for pkg in pkgs:
        if rosdep:
            if rosdep.has_ros(pkg):
                rosdep_pkgs.append(pkg)
                continue
        # TODO use python apt module to check if Debian package exists
        aptget_pkgs.append('-'.join(['ros', ros_distro, pkg.replace('_', '-')]))

    if rosdep_pkgs != []:
        try:
            cob_common.apt_get_install(rosdep_pkgs, rosdep, sudo)
        except:
            cob_common.BuildException("Failed to apt-get install rosdep packages.")

    if aptget_pkgs != []:
        try:
            cob_common.apt_get_install(aptget_pkgs, sudo=sudo)
        except:
            cob_common.BuildException("Failed to apt-get install ros repositories")
