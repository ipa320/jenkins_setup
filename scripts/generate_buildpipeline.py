#!/usr/bin/env python

import sys
import os
import optparse
import yaml
import jenkins

from jenkins_setup import jenkins_job_creator, cob_pipe


def get_master_name(url):
    name = url.replace('http://', '')
    name = name.replace(':8080/', '')
    name = name.replace(':8080', '')
    return name


# schedule cob buildpipeline jobs
def main():
    """
    Create a build and test pipeline on a Jenkins CI server.
    Starting point is the pipeline coniguration file (pipeline_config.yaml) of
    the given user stored on github. For this configuration a set of Jenkins
    projects/jobs will be generated.
    """

    # parse options
    usage = "Usage: %prog [masterURL login password configFolder tarballLocation | jenkinsConfigFile] pipelineReposOwner username"
    parser = optparse.OptionParser(usage)
    parser.add_option("-m", "--masterURL", action="store", type="string", dest="master_url",
                      metavar="URL", help="URL of Jenkins server/master")
    parser.add_option("-l", "--login", action="store", type="string", dest="jenkins_login",
                      metavar="LOGIN", help="Login name of Jenkins Admin which has rights to create and delete jobs")
    parser.add_option("-p", "--password", action="store", type="string", dest="jenkins_pw",
                      metavar="PASSWORD", help="Jenkins Admin password")
    parser.add_option("-c", "--configFolder", action="store", type="string", dest="config_folder",
                      metavar="CONFIGFOLDER", help="Folder where '.gitconfig', '.ssh', 'jenkins_setup' and 'jenkins_config' are stored")
    parser.add_option("-t", "--tarballLocation", action="store", type="string", dest="tarball_location",
                      metavar="USER@SERVERADDRESS:PATH", help="Place where the Tarballs are located")

    parser.add_option("--jenkinsConfigFile", action="store", type="string", dest="jenkinsConfigFile",
                      metavar="FILE", help="YAML file that replaces the Jenkins config options and contains values for:\
                                            masterURL, login, password, configFolder, tarballLocation")

    parser.add_option("-o", "--pipelineReposOwner", action="store", type="string", dest="pipeline_repos_owner",
                      metavar="PIPELINE_REPOS_OWNER", help="Owner of the GitHub repositories 'jenkins_setup' and 'jenkins_config'")
    parser.add_option("-u", "--username", action="store", type="string", dest="username",
                      metavar="USERNAME", help="Name of user to generate pipeline for")
    parser.add_option("-d", "--delete", action="store_true", default=False,
                      help="Delete")
    (options, args) = parser.parse_args()

    if len(args) != 0:
        print "Usage: %s [masterURL login password configFolder tarballLocation | jenkinsConfigFile] pipelineReposOwner username" % (sys.argv[0])
        sys.exit()

    if options.jenkinsConfigFile:
        # load jenkins config from file
        with open(os.path.expanduser(options.jenkinsConfigFile)) as f:
            jenkins_conf = yaml.load(f)

        master_name = get_master_name(jenkins_conf['masterURL'])
        tarball_location = jenkins_conf['tarballLocation']

        # create jenkins instance
        jenkins_instance = jenkins.Jenkins(jenkins_conf['masterURL'], jenkins_conf['login'],
                                           jenkins_conf['password'])

    elif options.master_url and options.jenkins_login and options.jenkins_pw and options.tarball_location:
        master_name = get_master_name(options.master_url)
        tarball_location = options.tarball_location

        # create jenkins instance
        jenkins_instance = jenkins.Jenkins(options.master_url, options.jenkins_login, options.jenkins_pw)

    else:
        print "Usage: %s [masterURL login password configFolder tarballLocation | jenkinsConfigFolder] pipelineReposOwner username" % (sys.argv[0])
        sys.exit()

    if not options.pipeline_repos_owner or not options.username:
        print "Usage: %s [masterURL login password configFolder tarballLocation | jenkinsConfigFolder] pipelineReposOwner username" % (sys.argv[0])
        sys.exit()

    # get all existent jobs for user
    existent_user_jobs = []
    for job in jenkins_instance.get_jobs():
        job_owner = job['name'].split('__')[0]
        if options.username == job_owner:
            existent_user_jobs.append(job['name'])
    modified_jobs = []

    # get pipeline configs object from url
    plc_instance = cob_pipe.CobPipe()
    plc_instance.load_config_from_url(options.pipeline_repos_owner, master_name, options.username)
    plc_instance.config_folder = options.config_folder

    # get jobs to create
    job_type_dict = plc_instance.get_jobs_to_create()

    ### create pipeline jobs
    ### pipe starter
    # for each repository and each polled user-defined dependency a pipe
    # starter job will be generated
    polls_dict = plc_instance.get_custom_dependencies(polled_only=True)
    pipe_repo_list = plc_instance.repositories.keys()
    for poll, starts_repo_list in polls_dict.iteritems():
        if poll in pipe_repo_list:
            pipe_repo_list.remove(poll)
        job_creator_instance = jenkins_job_creator.PipeStarterJob(jenkins_instance, plc_instance, starts_repo_list, poll)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())
    for repo in pipe_repo_list:
        job_creator_instance = jenkins_job_creator.PipeStarterJob(jenkins_instance, plc_instance, [repo], repo)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### manual pipe starter
    # this pipe starter job won't poll any repository; it has to be started
    # manually. It triggers the priority build job with a repositories from 
    # a pull down menu as parameter
    job_creator_instance = jenkins_job_creator.PipeStarterManualJob(jenkins_instance, plc_instance, plc_instance.repositories.keys())
    if options.delete:
        modified_jobs.append(job_creator_instance.delete_job())
    else:
        manual_pipe_starter_name = job_creator_instance.create_job()
        modified_jobs.append(manual_pipe_starter_name)

    ### general pipe starter
    # this pipe starter job won't poll any repository; it has to be started
    # manually. It triggers the priority build job with all defined
    # repositories as parameters
    job_creator_instance = jenkins_job_creator.PipeStarterGeneralJob(jenkins_instance, plc_instance, plc_instance.repositories.keys())
    if options.delete:
        modified_jobs.append(job_creator_instance.delete_job())
    else:
        general_pipe_starter_name = job_creator_instance.create_job()
        modified_jobs.append(general_pipe_starter_name)

    ### priority build
    job_creator_instance = jenkins_job_creator.PriorityBuildJob(jenkins_instance, plc_instance, tarball_location, plc_instance.repositories.keys())
    if options.delete:
        modified_jobs.append(job_creator_instance.delete_job())
    else:
        modified_jobs.append(job_creator_instance.create_job())

    ### regular build
    if 'regular_build' in job_type_dict:
        job_creator_instance = jenkins_job_creator.RegularBuildJob(jenkins_instance, plc_instance, tarball_location)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### downstream build
    #if 'downstream_build' in job_type_dict:
    #    job_creator_instance = jenkins_job_creator.DownstreamBuildJob(jenkins_instance, plc_instance, tarball_location, job_type_dict['downstream_build'])
    #    if options.delete:
    #        modified_jobs.append(job_creator_instance.delete_job())
    #    else:
    #        modified_jobs.append(job_creator_instance.create_job())

    ### priority nongraphics test
    if 'nongraphics_test' in job_type_dict:
        job_creator_instance = jenkins_job_creator.PriorityNongraphicsTestJob(jenkins_instance, plc_instance, tarball_location, job_type_dict['nongraphics_test'])
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### regular nongraphics test
    if 'nongraphics_test' in job_type_dict and 'regular_build' in job_type_dict:
        job_creator_instance = jenkins_job_creator.RegularNongraphicsTestJob(jenkins_instance, plc_instance, tarball_location, job_type_dict['nongraphics_test'])
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### priority graphics test
    if 'graphics_test' in job_type_dict:
        job_creator_instance = jenkins_job_creator.PriorityGraphicsTestJob(jenkins_instance, plc_instance, tarball_location, job_type_dict['graphics_test'])
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### regular graphics test
    if 'graphics_test' in job_type_dict and 'regular_build' in job_type_dict:
        job_creator_instance = jenkins_job_creator.RegularGraphicsTestJob(jenkins_instance, plc_instance, tarball_location, job_type_dict['graphics_test'])
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### hardware build and test
    if 'hardware_build' in job_type_dict:
        job_creator_instance = jenkins_job_creator.HardwareBuildTrigger(jenkins_instance, plc_instance)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

        job_creator_instance = jenkins_job_creator.HardwareBuildJob(jenkins_instance, plc_instance)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

        job_creator_instance = jenkins_job_creator.HardwareTestTrigger(jenkins_instance, plc_instance)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

        job_creator_instance = jenkins_job_creator.HardwareTestJob(jenkins_instance, plc_instance)
        if options.delete:
            modified_jobs.append(job_creator_instance.delete_job())
        else:
            modified_jobs.append(job_creator_instance.create_job())

    ### release job
    # TODO fix if statement
    #if ('release' and 'downstream_build' and 'nongraphics_test' and 'graphics_test'
    #        and 'hardware_build' and 'interactive_hw_test' in job_type_dict):
    #    print "Create release job"
        # TODO

    ### clean up
    # TODO

    # delete old and no more required jobs
    delete_msg = ""
    for job in [job for job in existent_user_jobs if job not in modified_jobs]:
        jenkins_instance.delete_job(job)
        delete_msg += "- %s\n" % job

    if delete_msg != "":
        print "Delete old and no more required jobs:\n" + delete_msg

    # start buildpipeline by general starter job
    #if options.run:
    #    jenkins_instance.build_job(general_pipe_starter_name)

if __name__ == "__main__":
    main()
