#!/usr/bin/env python

import optparse
import sys

from jenkins_setup import cob_common, cob_distro


def main():
    """
    Get the repositories to be triggered depending on the repositories with
    source code changes
    """
    # parse parameter values
    parser = optparse.OptionParser()
    (options, args) = parser.parse_args()

    if len(args) < 3:
        print "Usage: %s server_name user_name repo1 repo2 ..." % sys.argv[0]
        raise cob_common.BuildException("Wrong arguments for pipe_starter script")

    # get arguments
    server_name = args[0]
    user_name = args[1]
    # repositories with source code changes
    repo_list = [args[i] for i in range(2, len(args))]

    # (debug) output
    print "Pipe_starter job was triggered by a source code change in the following repositories:"
    for repo in repo_list:
        print " - ", repo

    # cob_distro_pipe object
    cdp_instance = cob_distro.Cob_Distro_Pipe()
    buildpipe_repos = cdp_instance.load_from_url(server_name, user_name)

    repos_to_trigger = []
    for repo in repo_list:
        # check if triggering repo is defined in buildpipe config and polled
        # TODO
        if repo in buildpipe_repos.repositories:
            repos_to_trigger.append(repo)
        elif repo in buildpipe_repos.get_custom_dependencies().keys():
            for parent_repo in buildpipe_repos.get_custom_dependencies()[repo]:
                if buildpipe_repos.repositories[parent_repo].dependencies[repo].poll:
                    repos_to_trigger.append(repo)
        else:
            raise cob_common.BuildException("Pipeline was triggered by repo %s which is \
                                             not in pipeline config!" % repo)

    return repos_to_trigger


if __name__ == "__main__":
    # global try
    try:
        return main()
        print "Pipe starter script finished cleanly!"

    # global catch
    except cob_common.BuildException as ex:
        print ex.msg

    except Exception as ex:
        print "Pipe starter script failed! Check out the console output above for details."
        raise ex
