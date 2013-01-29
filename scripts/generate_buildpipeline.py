#!/usr/bin/env python

import sys
import os
import optparse
import yaml
import jenkins

from jenkins_setup import jenkins_job_creator, cob_pipe


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

    # get all existent jobs for user
    existent_user_jobs = []
    for job in jenkins_instance.get_jobs():
        job_owner = job['name'].split('__')
        if user_name == job_owner:
            existent_user_jobs.append(job['name'])
    modified_jobs = []

    # get pipeline configs object from url
    plc_instance = cob_pipe.Cob_Pipe()
    plc_instance.load_config_from_url(info['master'], user_name)

    # get jobs to create
    job_type_dict = plc_instance.get_jobs_to_create()

    ### create pipeline jobs TODO
    ### pipe starter
    # for each repository and each polled user-defined dependency a pipe
    # starter job will be generated
    polls_dict = plc_instance.get_custom_dependencies(polled_only=True)
    pipe_repo_list = plc_instance.repositories.keys()
    for poll, starts_repo_list in polls_dict.iteritems():
        if poll in pipe_repo_list:
            pipe_repo_list.remove(poll)
        job_creator_instance = jenkins_job_creator.Pipe_Starter_Job(jenkins_instance, plc_instance, starts_repo_list, poll)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())
    for repo in pipe_repo_list:
        job_creator_instance = jenkins_job_creator.Pipe_Starter_Job(jenkins_instance, plc_instance, [repo], repo)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### general pipe starter
    # this pipe starter job won't poll any repository; it has to be started
    # manually. It triggers the priority build job with all defined
    # repositories as parameters
    job_creator_instance = jenkins_job_creator.Pipe_Starter_General_Job(jenkins_instance, plc_instance,
                                                                        plc_instance.repositories.keys())
    if options.delete:
        modified_jobs.append(job_creator_instance.delete_job())
    else:
        general_pipe_starter_name = job_creator_instance.create_job()
        modified_jobs.append(general_pipe_starter_name)

    ### priority build
    job_creator_instance = jenkins_job_creator.Priority_Build_Job(jenkins_instance, plc_instance)
    if options.delete:
        modified_jobs.append(job_creator_instance.delete_job())
    else:
        modified_jobs.append(job_creator_instance.create_job())

    ### normal build
    if 'normal' in job_type_dict:
        job_creator_instance = jenkins_job_creator.Normal_Build_Job(jenkins_instance, plc_instance)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### downstream build
    job_creator_instance = jenkins_job_creator.Downstream_Job(jenkins_instance, plc_instance)
    if options.delete:
        modified_jobs.append(job_creator_instance.delete_job())
    else:
        modified_jobs.append(job_creator_instance.create_job())

    ### database test

    ### simulation test

    ### application test

    ### bringup hardware test

    ### high-level hardware test

    ### release job
    if plc_instance.user_group == "admin":
        pass  # TODO

    ### clean up

    # delete old and no more required jobs
    print "Delete old and no more required jobs:"
    for job in [job for job in existent_user_jobs if job not in modified_jobs]:
        jenkins_instance.build_job(job)
        print "- %s" % job

    # start buildpipeline by general starter job
    #if options.run:
    #    jenkins_instance.build_job(general_pipe_starter_name)

if __name__ == "__main__":
    main()
