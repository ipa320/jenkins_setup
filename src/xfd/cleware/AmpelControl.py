#!/usr/bin/python



### TODO ###
# -make the jenkins url an input parameter
# -add clewarecontrol command to light up the ampel

import os
import sys
import urllib2
import time
from optparse import OptionParser
from bs4 import BeautifulSoup
from subprocess import call
from thread import start_new_thread

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
	

def get_jenkins_state(url):
	req = urllib2.urlopen(url)
	res = req.read()

	xml = BeautifulSoup(res)
	xml_colors = xml.find_all("color")

	state = []
	for color in xml_colors:
		state.append(str(color.string))
	if state == []:
		raise Exception("Received empty state. There is no color tag, check URL manually: " + url)
	return state

# color: (0=red, 1=yellow, 2=blue/green)
# mode: (0=off, 1=on, 2=flash)
def extract_ampel_state(state):
	message = "Ampel is "
	if "red_anime" in state:
		message += "red and flashing"
		state = [2,0,0]
	elif "red" in state:
		message += "red"
		state = [1,0,0]
	elif "yellow_anime" in state:
		message += "yellow and flashing"
		state = [0,2,0]
	elif "yellow" in state:
		message += "yellow"
		state = [0,1,0]
	elif "blue_anime" in state:
		message += "blue and flashing"
		state = [0,0,2]
	elif "blue" in state:
		message += "blue"
		state = [0,0,1]
	else:
		message = "Error: invalid state. state = " + str(state)
		state = [2,2,2]
	return state, message

def run_ampel():
	global ampel
	on = False
	while True:
		state = ampel.get_state()
		for color in [0,1,2]:
			if state[color] == 2 and not on:
				print "set on"
				#call(["clewarecontrol -as " + str(color) + " " + str(1)])
				on = True
			elif state[color] == 2 and on:
				print "set off"
				#call(["clewarecontrol -as " + str(0) + " " + str(0)])
				on = False
			else:
				print "set to state"
				#call(["clewarecontrol -as " + str(0) + " " + str(state[color])])
				pass
		time.sleep(1)
		print ""


if __name__ == "__main__":
	global ampel
	ampel = ampel_state()
	
	_usage = """%prog [options]
	type %prog -h for more info."""
	parser = OptionParser(usage=_usage, prog=os.path.basename(sys.argv[0]))
	parser.add_option("-u", "--url",
		dest="url", 
		default="http://build.care-o-bot.org:8080/view/All", 
		help="Jenkins server to monitor. Default: http://build.care-o-bot.org:8080")
	(options, args) = parser.parse_args()
	if len(args) != 0:
		parser.error("no arguments supported.")

	start_new_thread(run_ampel,())

	while True:
		try:
			jenkins_state = get_jenkins_state(options.url + "/api/xml")
			ampel_state, message = extract_ampel_state(jenkins_state)
			print message
			ampel.set_state(ampel_state)
		except Exception, e:
			print e
			pass
		time.sleep(3)
