"""
TODO
"""
import os
from jenkins_setup.common import apt_get_install, call


class RosDepResolver:
    """
    TODO
    """
    def __init__(self, ros_distro, sudo=False):
        self.r2a = {}
        self.a2r = {}
        self.env = os.environ
        self.env['ROS_DISTRO'] = ros_distro

        print "Ininitalize rosdep database"
        apt_get_install(['lsb-release', 'python-rosdep'], sudo=sudo)
        try:
            call("rosdep init", self.env)
        except:
            print "Rosdep is already initialized"
        call("rosdep update", self.env)

        print "Building dictionaries from a rosdep's db"
        raw_db = call("rosdep db", self.env, verbose=False).split('\n')

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
        res = []
        for r_e in ros_entries:
            for apt in self.to_apt(r_e):
                if not apt in res:
                    res.append(apt)
        return res

    def to_ros(self, apt_entry):
        if apt_entry not in self.a2r:
            print "Could not find %s in rosdep keys. Rosdep knows about these keys: %s" % (apt_entry, ', '.join(self.a2r.keys()))
        return self.a2r[apt_entry]

    def to_apt(self, ros_entry):
        if ros_entry not in self.r2a:
            print "Could not find %s in keys. Have keys %s" % (ros_entry, ', '.join(self.r2a.keys()))
        return self.r2a[ros_entry]

    def has_ros(self, ros_entry):
        return ros_entry in self.r2a

    def has_apt(self, apt_entry):
        return apt_entry in self.a2r


class RosDep:
    """
    TODO
    """
    def __init__(self, ros_distro):
        self.r2a = {}
        self.a2r = {}
        self.env = os.environ
        self.env['ROS_DISTRO'] = ros_distro

        # Initialize rosdep database
        print "Ininitalize rosdep database"
        call("apt-get install --yes lsb-release python-rosdep")
        call("rosdep init", self.env)
        call("rosdep update", self.env)

    def to_apt(self, ros):
        if ros in self.r2a:
            return self.r2a[ros]
        else:
            res = call("rosdep resolve %s" % ros, self.env).split('\n')
            if len(res) == 1:
                raise Exception("Could not resolve rosdep")
            apt = call("rosdep resolve %s" % ros, self.env).split('\n')[1]
            print "Rosdep %s resolved into %s" % (ros, apt)
            self.r2a[ros] = apt
            self.a2r[apt] = ros
            return apt

    def to_stack(self, apt):
        if not apt in self.a2r:
            print "%s not in apt-to-rosdep cache" % apt
        return self.a2r[apt]
