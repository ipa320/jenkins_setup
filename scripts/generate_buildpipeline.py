#!/usr/bin/env python

import sys
import os
import optparse
import yaml
import jenkins

from rospkg import environment
from jenkins_setup import run_jenkins_job_creation, cob_common
#from jenkins_tools.run_jenkins_job_creation import *


# schedule cob buildpipeline jobs
def main():
    # parse options
    parser = optparse.OptionParser()
    parser.add_option("--run", action="store_true", default=False)  # TODO
    (options, args) = parser.parse_args()

    if len(args) != 3:  # TODO
        print "Usage: %s username servername" % (sys.argv[0])
        sys.exit()

    user_name = args[0]
    server_name = args[1]

    # create jenkins instance TODO
    with open(os.path.join(environment.get_ros_home(), 'catkin-debs', 'server.yaml')) as f:
        info = yaml.load(f)
    jenkins_instance = jenkins.Jenkins(run_jenkins_job_creation.JENKINS_SERVER, info['username'],
                                       info['password'])  # TODO

    # get pipeline configs
    bpl_configs = cob_common.get_buildpipeline_configs('__'.join[user_name, server_name])

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
