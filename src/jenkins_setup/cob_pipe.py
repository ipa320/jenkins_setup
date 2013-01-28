#!/usr/bin/env python


from jenkins_setup import cob_distro, cob_common


class Cob_Pipe(cob_distro.Cob_Distro_Pipe):
    """
    Pipeline configuration class
    """

    def load_config_from_dict(self, pipeline_config):
        """
        Sets up a pipeline object derived from the given dictionary

        @param pipeline_config: pipeline configuration
        @type  pipeline_config: dict
        """

        self.pipeline_github_url = pipeline_config['pipeline_github_url']
        self.user_name = pipeline_config['user_name']
        self.user_group = pipeline_config['user_group']
        self.server_name = pipeline_config['server_name']
        self.email = pipeline_config['email']
        self.committer = pipeline_config['committer']

        super(Cob_Pipe, self).load_config_from_dict(pipeline_config['repositories'])

    def load_config_from_url(self, server_name, user_name):
        """
        Gets the buildpipeline configuration by the given server and user name
        and sets up the pipeline object

        @param server_name: name of server
        @type  server_name: str
        @param user_name: name of user
        @type  user_name: str
        """

        pipeline_config = cob_common.get_buildpipeline_configs(server_name, user_name)
        self.load_config_from_dict(pipeline_config)

    def get_jobs_to_create(self):
        """
        Gets a list of all job types to create
        """

        job_types = {}
        for repo in self.repositories.keys():
            for job in self.repositories[repo].jobs:
                job_types[job] = 1

        return job_types.keys()
