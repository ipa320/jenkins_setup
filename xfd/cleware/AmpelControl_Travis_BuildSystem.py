#!/usr/bin/python

import os
import sys
import time
import subprocess
from optparse import OptionParser
from thread import start_new_thread
import travispy
from pygithub3 import Github

class ampel_state:
    def __init__(self):
        # modes
        self.off = 0
        self.on = 1
        self.flash = 2
        # state
        self.state = [self.off,self.off,self.off]

    def set_state(self, state):
        self.state = state

    def get_state(self):
        return self.state

def extract_ampel_state(state):
    message = "Ampel is "
    state_out = [0,0,0]
    if "red_anime" in state:
        message += "flashing red"
        state_out[0] = 2
    elif "red" in state:
        message += "red"
        state_out[0] = 1
    else:
        message += "no red"
        state_out[0] = 0

    message += ", "
    if "yellow_anime" in state:
        message += "flashing yellow"
        state_out[1] = 2
    elif "yellow" in state:
        message += "yellow"
        state_out[1] = 1
    else:
        message += "no yellow"
        state_out[1] = 0

    message += " and "
    if "green_anime" in state:
        message += "flashing green"
        state_out[2] = 2
    elif "green" in state:
        message += "green"
        state_out[2] = 1
    else:
        message += "no green"
        state_out[2] = 0

    if state_out == [0, 0, 0]:
        message = "Error: invalid state. state = " + str(state)
        state_out = [2,2,2]
    return state_out, message

def run_ampel():
    global ampel
    on = [False, False, False]
    default_options = ["-c", "1"]
    if options.device != None:
        default_options += ["-d", str(options.device)]
    while True:
        state = ampel.get_state()
        for color in [0,1,2]:
            if state[color] == 2 and not on[color]:
                p = subprocess.Popen(["clewarecontrol"] + default_options + ["-as", str(color), "1"], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
                on[color] = True
            elif state[color] == 2 and on[color]:
                p = subprocess.Popen(["clewarecontrol"] + default_options + [ "-as", str(color), "0"], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
                on[color] = False
            else:
                p = subprocess.Popen(["clewarecontrol"] + default_options + [ "-as", str(color), str(state[color])], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
            p.wait()
        time.sleep(1)


def extract_build_query(file_path):
    tag_store = []
    with open(file_path) as fp:
        for line in fp:
            if "git:" in line:
                user = None
                repo = None
                branch = None
            elif "uri:" in line:
                user = line.split("github.com",1)[1]
                user = user[1:]
                user = user.split("/", 1)[0].strip()
            elif "local-name:" in line:
                repo = line.split(":",1)[1].strip()
            elif "version:" in line:
                branch = line.split(":",1)[1].strip()
                build_info_tag = {
                    "user":    user,
                    "repo":    repo,
                    "branch":  branch,
                }
                tag_store.append(build_info_tag)
            else:
                pass
        fp.close()
    return tag_store
    

def extract_travis_build_states(path, t_public, t_private):
    state_store = []
    build_query_data = extract_build_query(options.path)

    build_states = []
    for data in build_query_data:
        print "repo:", data['user'] + '/' + data['repo']
        invalid_state = False
        # try public server
        try:
            branch = t_public.branch(data['branch'], data['user'] + '/' + data['repo'])
            build_states.append(branch.state)
            t = t_public
        except travispy.errors.TravisError, e:
            # try private server
            try:
                #print "no public repo, trying private url. error:", e
                branch = t_private.branch(data['branch'], data['user'] + '/' + data['repo'])
                build_states.append(branch.state)
                t = t_private
            except Exception, e:
                invalid_state = True
        except Exception, e:
            invalid_state = True

        # check for invalid state
        if invalid_state:
            print "error for " + data['user'] + '/' + data['repo']  + " in branch " + data['branch'] + ": ", e
            state_store.append('red_anime')
            state_store.append('yellow_anime')
            state_store.append('green_anime')
            return state_store

        # check PRs
        gh = Github(token=options.token, user=data['user'], repo=data['repo'])
        gh_prs = []
        prs = t.builds(slug=data['user'] + '/' + data['repo'], event_type="pull_request")
        for gh_pr in gh.pull_requests.list().all():
            #print "gh_pr:", gh_pr.number, gh_pr.title
            gh_prs.append(gh_pr.number)
            for pr in prs:
                if pr.pull_request_number == gh_pr.number:
                    print "--> open pr:", pr.pull_request_number, pr.pull_request_title, pr.state
                    if pr.state == "passed":
                        print "----> merge"
                        state_store.append('yellow_anime')
                    else:
                        print "----> review"
                        state_store.append('yellow')
                    break

    print "build_states", build_states

    ### set red ###
    # check for failed builds
    if 'failed' in build_states or 'errored' in build_states or 'canceled' in build_states:
        state_store.append("red")
    
    ### set yellow ###
    # check for ready to merge PRs
    # TODO (flash yellow)
        
    ### set green ###
    # check for ongoing builds
    if 'created' in build_states or 'started' in build_states or 'queued' in build_states:
        state_store.append("green_anime")
    # all builds ok
    if 'red' not in state_store and 'red_amine' not in state_store:
        state_store.append("green")

    return state_store


if __name__ == "__main__":
    global ampel
    ampel = ampel_state()

    _usage = """%prog [options]
    type %prog -h for more info."""
    parser = OptionParser(usage=_usage, prog=os.path.basename(sys.argv[0]))
    parser.add_option("-p", "--path",
        dest="path",
        default=None,
        help="The ROSInstall file path for parsing repositories URLs to monitor build states.")
    parser.add_option("-t", "--token",
        dest="token",
        default=None,
        help="Please give github token, this is different than one found on your github profile")
    parser.add_option("-d", "--device",
        dest="device",
        default=None,
        help="Use device with serial number 'x' for the next operations")
    (options, args) = parser.parse_args()
    if len(args) != 0:
        parser.error("no arguments supported.")

    start_new_thread(run_ampel,())
    # initial reset the ample states
    ampel_state = [0,0,0]
    ampel.set_state(ampel_state)


    t_public = travispy.TravisPy.github_auth(options.token, travispy.travispy.PUBLIC)
    t_private = travispy.TravisPy.github_auth(options.token, travispy.travispy.PRIVATE)

    while True:
        try:
            travis_state = extract_travis_build_states(options.path, t_public, t_private)
            ampel_state, message = extract_ampel_state(travis_state)
            print "\n[",message,"]" , "Ample States:",ampel_state
            ampel.set_state(ampel_state)
        except Exception, e:
            print e
            ampel.set_state([2, 2, 2])
            pass
        time.sleep(10)

