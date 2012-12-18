#!/usr/bin/env python

import sys
import os
import optparse
import urllib
import yaml
import jenkins

from rospkg import environment
from jenkins_setup import run_jenkins_job_creation
#from jenkins_tools.run_jenkins_job_creation import *


# schedule cob buildpipeline jobs
def main():
    # parse options
    parser = optparse.OptionParser()
    parser.add_option("--run", action="store_true", default=False)  # TODO
    (options, args) = parser.parse_args()

    if len(args) != 2:  # TODO
        print "Usage: %s github_url" % (sys.args[0])
        sys.exit()

    github_config_url = args[0]

    # create jenkins instance TODO
    with open(os.path.join(environment.get_ros_home(), 'catkin-debs', 'server.yaml')) as f:
        info = yaml.load(f)
    jenkins_instance = jenkins.Jenkins(run_jenkins_job_creation.JENKINS_SERVER, info['username'],
                                       info['password'])  # TODO

    # parse yaml file
    print "Parsing buildpipeline configuration yaml file from %s" % github_config_url
    f = urllib.urlopen(github_config_url)
    bpl_configs = yaml.load(f.read())  # TODO

    # create pipeline jobs TODO
    pipe_starter_name = run_jenkins_job_creation.create_pipe_starter()

    # create release job TODO
    if bpl_configs['user_group'] == "admin":
        run_jenkins_job_creation.create_release()

    # start buildpipeline by build pipe starter job TODO
    if options.run:
        jenkins_instance.build_job(pipe_starter_name)

if __name__ == "__main__":
    main()
