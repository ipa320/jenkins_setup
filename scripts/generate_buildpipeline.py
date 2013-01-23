#!/usr/bin/env python

import sys
import os
import optparse
import yaml
import jenkins

from jenkins_setup import jenkins_job_creator, cob_common, cob_distro


# schedule cob buildpipeline jobs
def main():
    # parse options
    parser = optparse.OptionParser()
    parser.add_option("--run", action="store_true", default=False)  # TODO
    (options, args) = parser.parse_args()

    if len(args) != 1:  # TODO
        print "Usage: %s username" % (sys.argv[0])
        sys.exit()

    user_name = args[0]

    # load slave config
    with open(os.path.expanduser('~/jenkins-config/slave_config.yaml')) as f:
        info = yaml.load(f)

    # create jenkins instance
    jenkins_instance = jenkins.Jenkins(info['master_url'], info['jenkins_login'],
                                       info['jenkins_pw'])

    # get pipeline configs
    bpl_configs = cob_common.get_buildpipeline_configs(info['master'], user_name)
    # get pipeline configs object and load configs from bpl_configs dict
    bplc_instance = cob_distro.Cob_Distro_Pipe()
    bplc_instance.load_from_dict(bpl_configs['repositories'])

    #job_creator_instance = jenkins_job_creator.Jenkins_Jobs(jenkins_instance, bpl_configs)

    # create pipeline jobs TODO
    # pipe starter
    polls_dict = bplc_instance.get_custom_dependencies(polled_only=True)
    print polls_dict
    for poll, repo_list in polls_dict.iteritems():
        print poll, repo_list
        job_creator_instance = jenkins_job_creator.Pipe_Starter_Job(jenkins_instance, bpl_configs, repo_list, poll)
        job_creator_instance.create_job()
    for repo in bplc_instance.repositories.keys():
        print repo
        job_creator_instance = jenkins_job_creator.Pipe_Starter_Job(jenkins_instance, bpl_configs, [repo], repo)
        job_creator_instance.create_job()

    # general pipe starter
    job_creator_instance = jenkins_job_creator.Pipe_Starter_General_Job(jenkins_instance, bpl_configs,
                                                                        bplc_instance.repositories.keys())
    job_creator_instance.create_job()

    # priority build

    # create release job TODO
    if bpl_configs['user_group'] == "admin":
        pass

    # start buildpipeline by build pipe starter job TODO
    #if options.run:
    #    jenkins_instance.build_job(pipe_starter_name)

if __name__ == "__main__":
    main()
