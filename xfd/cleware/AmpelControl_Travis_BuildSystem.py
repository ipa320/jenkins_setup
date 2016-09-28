#!/usr/bin/python

import os
import sys
import urllib2
import time
import subprocess
from optparse import OptionParser
from thread import start_new_thread
import json


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


# color: (0=red, 1=yellow, 2=blue/green)
# mode: (0=off, 1=on, 2=flash)
# red: if build state Failure/Error for at least one repo
# yellow flash: if at least one repo is Active
# green: if all repo build state Success
# all flash: if any of the build url is invalid/no internet connection
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
	if "blue_anime" in state or "grey_anime" in state:
		message += "flashing green"
		state_out[2] = 2
	elif "blue" in state:
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
            #print line
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
    

def extract_travis_build_states(path, auth_token):
    '''
    :construct URI to get minimalistic information 
    regarding the build-status of each repos
    :We could not identify if the repo is a private or public one,
    modify URI for public/private repo and get build status data
    '''
    url_string = None
    url = "https://api.travis-ci."
    private_domain = "com"
    public_domain = "org"
    
    build_query_data = extract_build_query(path)
    state_store = []
    
    checker_build_failure = False
    checker_repo_active = False
    checker_invalid_state = False
    
    print "\nChecking build states for repositories....\n"
    
    ### iterate over all the repos and get their travis build states
    for data in build_query_data:
        owner = data["user"]
        repo_name = data["repo"]
        branch = data["branch"]
        ### on valid owner, repo_name and branch, fire the urls and get travis buld states
        if owner != "" and repo_name != "" and branch != "":
            print "[Owner:",owner,"]  [Repository:",repo_name,"]  [Branch:",branch,"]"
            ### first try to get json build data assuming the repo is public
            try:
                url_string = url+public_domain+"/repos/"+owner+'/'+repo_name+".json?branch="+branch 
                response_data = urllib2.urlopen(url_string)
            except urllib2.HTTPError, e:
                response_data = ""
                checker_invalid_state = True
                print e,", trying private url"
            except urllib2.URLError, e:
                response_data = ""
                checker_invalid_state = True
                print e,", trying private url"
            ### second try to get json build data assuming the repo is private
            if response_data == "":
                try:
                    url_string = url+private_domain+"/repos/"+owner+'/'+repo_name+".json?token="+auth_token+"&branch="+branch
                    response_data = urllib2.urlopen(url_string)
                except urllib2.HTTPError, e:
                    response_data = ""
                    checker_invalid_state = True
                    print e, "-------------------------------------------"
                except urllib2.URLError, e:
                    response_data = ""
                    checker_invalid_state = True
                    print e, "-----------------------------------------------"
            ### if success in getting response (from public/private repo), we get valid build states, now check each state result
            if response_data != "":
                checker_invalid_state = False
                build_data = json.loads(response_data.read())
                for entry in build_data.items():
                    if entry[0]:
                        #if entry[0] == "last_build_status":
                        if entry[0] == "last_build_result":
                            if entry[1] == 1: ### build falied/error
                                checker_build_failure = True
                                print "---> Build failed for repository:", repo_name
                        if entry[0] == "last_build_duration":
                            if entry[1] == None: ### repo is active
                                checker_repo_active = True
                                print "---> Active repository:", repo_name
                    else:
                        print "No build state found for:"
                        print "[Owner*:",owner,"]  [Repository*:",repo_name,"]  [Branch*:",branch,"]"
                        ### if an invalid state is encountered (possibly wrong url), quickly alert
            if checker_invalid_state:
                break
        else:
            checker_invalid_state = True
            print "Found invalid owner/repository/branch name, please check the supplied links:"
            print "[Owner*:",owner,"]  [Repository*:",repo_name,"]  [Branch*:",branch,"]" 
            
    ### varify the build states and set flags
    if checker_invalid_state:
        print "\nThere is an invalid build url for repository:",repo_name," Please check manually !!\n"
        state_store.append("blue_anime")
        state_store.append("red_anime")
        state_store.append("yellow_anime")
    else:
        if checker_build_failure:
            state_store.append("red")
        else:
            state_store.append("blue")
        if checker_repo_active:
            state_store.append("yellow_anime")
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
	if options.token == None:
	    options.token = "8SejjXqqq1WHebgRttQF"  ### assigning a default token

	while True:
	    try:
	        travis_state = extract_travis_build_states(options.path, options.token)
	        ampel_state, message = extract_ampel_state(travis_state)
	        print "\n[",message,"]" , "Ample States:",ampel_state
	        ampel.set_state(ampel_state)
	    except Exception, e:
	        print e
	        ampel.set_state([2, 2, 2])
	        pass
	    time.sleep(10)

