"""
This module provides the classes RosDepResolver and RosDep to get access to
Rosdep. The Rosdep database can be used to find for ROS stacks the corresponding
apt packages and the other way around.
"""
import os
from jenkins_setup.common import apt_get_install, call


class RosDepResolver(object):

    """
    This class allows to initialize a rosdep database and provide the access to
    it the find corresponding ROS stacks and apt packages. Even a list of ROS
    stacks can be converted into a list of apt packages.
    """

    def __init__(self, ros_distro, sudo=False):
        """
        Initializes rosdep database and builds necessary dictionaries

        ros_distro -- string of ROS version
        sudo -- whether commands should be executed with sudo (default False)
        """
        self.r2a = {}
        self.a2r = {}
        self.env = os.environ
        self.env['ROS_DISTRO'] = ros_distro

        print "Initialize rosdep database"
        #apt_get_install(['lsb-release', 'python-rosdep'], sudo=sudo)
        try:
            call("rosdep init", self.env)
        except:
            print "Rosdep is already initialized"
        call("rosdep update", self.env)

        print "Building dictionaries from a rosdep's db"
        out, err = call("rosdep db", self.env, verbose=False)
        raw_db = out.split('\n')

        for entry in raw_db:
            split_entry = entry.split(' -> ')
            if len(split_entry) < 2:
                continue
            ros_entry = split_entry[0]
            apt_entries = split_entry[1].split(' ')
            self.r2a[ros_entry] = apt_entries
            for a_e in apt_entries:
                self.a2r[a_e] = ros_entry

    def to_aptlist(self, ros_entries):
        """
        Finds for a list of ros entries their corresponding apt entries.

        ros_entries -- list of ros entries
        return -- list of corresponding apt entries
        """
        res = []
        for r_e in ros_entries:
            for apt in self.to_apt(r_e):
                if not apt in res:
                    res.append(apt)
        return res

    def to_ros(self, apt_entry):
        """
        Tries to find ros entry for given apt entry.

        apt_entry -- apt entry to find ros entry for
        return -- string of ros entry
        """
        if apt_entry not in self.a2r:
            print "Could not find %s in rosdep keys. Rosdep knows about these keys: %s" % (apt_entry, ', '.join(self.a2r.keys()))
        return self.a2r[apt_entry]

    def to_apt(self, ros_entry):
        """
        Tries to find apt entry for given ros entry.

        ros_entry -- ros entry to find apt entry for
        return -- string of apt entry
        """
        if ros_entry not in self.r2a:
            print "Could not find %s in keys. Have keys %s" % (ros_entry, ', '.join(self.r2a.keys()))
        return self.r2a[ros_entry]

    def has_ros(self, ros_entry):
        """
        Checks if the given ros entry is available.

        ros_entry -- ros entry to search for
        return -- boolean
        """
        return ros_entry in self.r2a

    def has_apt(self, apt_entry):
        """
        Checks if the given apt entry is available.

        apt_entry -- apt entry to search for
        return -- boolean
        """
        return apt_entry in self.a2r


class RosDep(object):

    """
    This class allows to initialize a rosdep database and provide the access to
    it the find corresponding ROS stacks and apt packages. Even a list of ROS
    """

    def __init__(self, ros_distro):
        """
        Installs and initializes the rosdep database.

        ros_distro -- string of ROS version
        """
        self.r2a = {}
        self.a2r = {}
        self.env = os.environ
        self.env['ROS_DISTRO'] = ros_distro

        # Initialize rosdep database
        print "Initalize rosdep database"
        call("apt-get install --yes lsb-release python-rosdep")
        call("rosdep init", self.env)
        call("rosdep update", self.env)

    def to_apt(self, ros):
        """
        Tries to find apt package for given ros stack.

        ros -- string of ros stack
        return -- string of apt package
        """
        if ros in self.r2a:
            return self.r2a[ros]
        else:
            out, err = call("rosdep resolve %s" % ros, self.env)
            res = out.split('\n')
            if len(res) == 1:
                raise Exception("Could not resolve rosdep")
            out, err = call("rosdep resolve %s" % ros, self.env)
            apt = out.split('\n')[1]
            print "Rosdep %s resolved into %s" % (ros, apt)
            self.r2a[ros] = apt
            self.a2r[apt] = ros
            return apt

    def to_stack(self, apt):
        """
        Tries to find ros stack for given apt package.

        apt -- string of apt package
        return -- string of ros stack
        """
        if not apt in self.a2r:
            print "%s not in apt-to-rosdep cache" % apt
        return self.a2r[apt]
