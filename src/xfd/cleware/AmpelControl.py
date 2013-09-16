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


def get_state(url):
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

# color: (0=red, 1=yellow, 2=blue)
# mode: (0=off, 1=on, 2=flash)
def set_ampel(state):
	message = "Ampel is "
	if "red_anime" in state:
		message += "red and flashing"
		set_cleware(0, 2)
	elif "red" in state:
		message += "red"
		set_cleware(0, 1)
	elif "yellow_anime" in state:
		message += "yellow and flashing"
		set_cleware(1, 2)
	elif "yellow" in state:
		message += "yellow"
		set_cleware(1, 1)
	elif "blue_anime" in state:
		message += "blue and flashing"
		set_cleware(2, 2)
	elif "blue" in state:
		message += "blue"
		set_cleware(2, 1)
	else:
		message = "Error: invalid state. state = " + str(state)
		set_cleware("all",2)
	return message


def set_cleware(color,mode):
	print "setting ampel to color: " + str(color) + " in mode: " + str(mode) # TODO: remove
	#TODO: add clewarecontrol commands here
	#clewarecontrol -as color, mode
	


if __name__ == "__main__":
	
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

	while True:
		try:
			state = get_state(options.url + "/api/xml")
			print options.url + ": " + set_ampel(state)
		except Exception, e:
			print e
			pass
		time.sleep(1)
