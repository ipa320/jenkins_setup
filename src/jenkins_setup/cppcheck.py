#!/usr/bin/env python

import subprocess
from jenkins_setup import common

def run(source_path, result_path):
    print "===== Performing cppcheck ====="

    if type(source_path) is str:
        paths = [source_path]
    elif type(source_path) is list:
        paths = source_path
    else:
        raise common.BuildException("cppcheck source_path neiter string nor list")

    try: 
        out, err = common.call("cppcheck --enable=all -j8 -q --xml " + " ".join(paths), verbose=False)
        common.call("touch " + result_path + "/cppcheck.xml")
        f = open(result_path + "/cppcheck.xml", 'w')
        f.write(err)
    except:
        print "Error in cppcheck. Skipping test, no results generated."

    print "===== Completed cppcheck ====="

if __name__ == "__main__":
    run("/tmp/src","/tmp/result")
    #run(["/tmp/src","/tmp/src2"],"/tmp/result")
