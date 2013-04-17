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

    if len(args) != 2:
        print "Usage: %s config_repo_address username" % (sys.argv[0])
        sys.exit()

    config_repo = args[0]
    user_name = args[1]

    # load slave config
    with open(os.path.expanduser('~/jenkins-config/slave_config.yaml')) as f:
        slave_conf = yaml.load(f)

    # create jenkins instance
    jenkins_instance = jenkins.Jenkins(slave_conf['master_url'], slave_conf['jenkins_login'],
                                       slave_conf['jenkins_pw'])

    # get all existent jobs for user
    existent_user_jobs = []
    for job in jenkins_instance.get_jobs():
        job_owner = job['name'].split('__')[0]
        if user_name == job_owner:
            existent_user_jobs.append(job['name'])
    modified_jobs = []

    # get pipeline configs object from url
    plc_instance = cob_pipe.CobPipe()
    plc_instance.load_config_from_url(config_repo, slave_conf['master'], user_name)

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
        job_creator_instance = jenkins_job_creator.PipeStarterJob(jenkins_instance, plc_instance, starts_repo_list, poll, ['automatic_hw_test', 'interactive_hw_test'])  # TODO
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())
    for repo in pipe_repo_list:
        job_creator_instance = jenkins_job_creator.PipeStarterJob(jenkins_instance, plc_instance, [repo], repo, ['automatic_hw_test', 'interactive_hw_test'])  # TODO
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### general pipe starter
    # this pipe starter job won't poll any repository; it has to be started
    # manually. It triggers the priority build job with all defined
    # repositories as parameters
    job_creator_instance = jenkins_job_creator.PipeStarterGeneralJob(jenkins_instance, plc_instance,
                                                                     plc_instance.repositories.keys(), ['automatic_hw_test', 'interactive_hw_test'])  # TODO
    if options.delete:
        modified_jobs.append(job_creator_instance.delete_job())
    else:
        general_pipe_starter_name = job_creator_instance.create_job()
        modified_jobs.append(general_pipe_starter_name)

    ### priority build
    job_creator_instance = jenkins_job_creator.PriorityBuildJob(jenkins_instance, plc_instance, plc_instance.repositories.keys())
    if options.delete:
        modified_jobs.append(job_creator_instance.delete_job())
    else:
        modified_jobs.append(job_creator_instance.create_job())

    ### regular build
    if 'regular_build' in job_type_dict:
        job_creator_instance = jenkins_job_creator.RegularBuildJob(jenkins_instance, plc_instance)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### downstream build
    if 'downstream_build' in job_type_dict:
        job_creator_instance = jenkins_job_creator.DownstreamBuildJob(jenkins_instance, plc_instance, job_type_dict['downstream_build'])
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### nongraphics test
    if 'downstream_build' and 'nongraphics_test' in job_type_dict:
        job_creator_instance = jenkins_job_creator.NongraphicsTestJob(jenkins_instance, plc_instance, job_type_dict['nongraphics_test'])
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### graphics test
    if 'downstream_build' and 'graphics_test' in job_type_dict:
        job_creator_instance = jenkins_job_creator.GraphicsTestJob(jenkins_instance, plc_instance, job_type_dict['graphics_test'])
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### hardware build
    if 'hardware_build' in job_type_dict:
        job_creator_instance = jenkins_job_creator.HardwareBuildJob(jenkins_instance, plc_instance)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### automatic hardware test
    if 'hardware_build' and 'automatic_hw_test' in job_type_dict:
        job_creator_instance = jenkins_job_creator.AutomaticHWTestJob(jenkins_instance, plc_instance)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### interactive hardware test
    if 'hardware_build' and 'interactive_hw_test' in job_type_dict:
        job_creator_instance = jenkins_job_creator.InteractiveHWTestJob(jenkins_instance, plc_instance)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### release job
    if 'downstream_build' and 'nongraphics_test' and 'graphics_test' and 'automatic_hw_test' and 'interactive_hw_test' in job_type_dict:
        print "Create release job"  # TODO

    ### clean up

    # delete old and no more required jobs
    print "Delete old and no more required jobs:"
    for job in [job for job in existent_user_jobs if job not in modified_jobs]:
        jenkins_instance.delete_job(job)
        print "- %s" % job

    # start buildpipeline by general starter job
    #if options.run:
    #    jenkins_instance.build_job(general_pipe_starter_name)

if __name__ == "__main__":
    main()
