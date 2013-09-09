#!/usr/bin/python



### TODO ###
# -make the jenkins url an input parameter
# -add clewarecontrol command to light up the ampel

import urllib2
import time
from bs4 import BeautifulSoup


def get_state(url):
	req = urllib2.urlopen(url)
	res = req.read()

	xml = BeautifulSoup(res)
	xml_colors = xml.find_all("color")

	state = []
	for color in xml_colors:
		state.append(str(color.string))
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
		message = "Error: invalid status."
		set_cleware("all",2)
	return message


def set_cleware(color,mode):
	print "setting ampel to color: " + str(color) + " in mode: " + str(mode) # TODO: remove
	#TODO: add clewarecontrol commands here
	#clewarecontrol -as color, mode
	


if __name__ == "__main__":

	while True:
		#url = 'http://localhost:8080/view/my_view/api/xml'
		url = 'http://cob-jenkins-server:8080/view/u_320/api/xml'
		state = get_state(url)
	
		print set_ampel(state)
		time.sleep(1)
	
