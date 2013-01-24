#!/usr/bin/env python

import sys
import os
import optparse
import yaml
import jenkins

from jenkins_setup import jenkins_job_creator, cob_common, cob_distro


# schedule cob buildpipeline jobs
def main():
    """
    Create a build and test pipeline on a Jenkins CI server.
    Starting point is the pipeline coniguration file (pipeline_config.yaml) of
    the given user stored on github. For this configuration a set of Jenkins
    projects/jobs will be generated.
    """

    # parse options
    parser = optparse.OptionParser()
    parser.add_option("--delete", action="store_true", default=False)
    (options, args) = parser.parse_args()

    if len(args) != 1:
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
    pl_configs = cob_common.get_buildpipeline_configs(info['master'], user_name)
    # get pipeline configs object and load configs from bpl_configs dict
    plc_instance = cob_distro.Cob_Distro_Pipe()
    plc_instance.load_from_dict(pl_configs['repositories'])

    #job_creator_instance = jenkins_job_creator.Jenkins_Jobs(jenkins_instance, bpl_configs)

    ### create pipeline jobs TODO
    ### pipe starter
    # for each repository and each polled user-defined dependency a pipe
    # starter job will be generated
    polls_dict = plc_instance.get_custom_dependencies(polled_only=True)
    for poll, repo_list in polls_dict.iteritems():
        job_creator_instance = jenkins_job_creator.Pipe_Starter_Job(jenkins_instance, pl_configs, repo_list, poll)
        if options.delete:
            job_creator_instance.delete_job()
        else:
            job_creator_instance.create_job()
    for repo in plc_instance.repositories.keys():
        job_creator_instance = jenkins_job_creator.Pipe_Starter_Job(jenkins_instance, pl_configs, [repo], repo)
        if options.delete:
            job_creator_instance.delete_job()
        else:
            job_creator_instance.create_job()

    ### general pipe starter
    # this pipe starter job won't poll any repository; it has to be started
    # manually. It triggers the priority build job with all defined
    # repositories as parameters
    job_creator_instance = jenkins_job_creator.Pipe_Starter_General_Job(jenkins_instance, pl_configs,
                                                                        plc_instance.repositories.keys())
    if options.delete:
        job_creator_instance.delete_job()
    else:
        job_creator_instance.create_job()

    ### priority build
    job_creator_instance = jenkins_job_creator.Priority_Build_Job(jenkins_instance, pl_configs)
    if options.delete:
        job_creator_instance.delete_job()
    else:
        job_creator_instance.create_job()

    ### create release job
    if pl_configs['user_group'] == "admin":
        pass  # TODO

    # start buildpipeline by build pipe starter job TODO
    #if options.run:
    #    jenkins_instance.build_job(pipe_starter_name)

if __name__ == "__main__":
    main()
