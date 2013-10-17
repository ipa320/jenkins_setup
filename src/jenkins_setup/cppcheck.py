#!/usr/bin/env python

import subprocess
from jenkins_setup import common

def run(source_path, result_path):
    print "===== Performing CPP check ====="

    common.apt_get_install(["cppcheck"]) # TODO: move this to basic tarball
    print source_path
    out, err = common.call("cppcheck --enable=all -j8 -q --xml " + str(source_path), verbose=False)
    common.call("touch " + result_path + "/cppcheck.xml")
    f = open(result_path + "/cppcheck.xml", 'w')
    f.write(err)

    print "===== Completed CPP check ====="

if __name__ == "__main__":
    run("/tmp/src","/tmp/result")
