#!/usr/bin/python


import urllib2
import time
from bs4 import BeautifulSoup


def get_colors(url):
	req = urllib2.urlopen(url)
	res = req.read()

	xml = BeautifulSoup(res)
	xml_colors = xml.find_all("color")

	colors = []
	for color in xml_colors:
		colors.append(str(color.string))
	return colors

# color: (0=red, 1=yellow, 2=blue)
# mode: (0=off, 1=on, 2=flash)
# returns [color,mode]
def get_state(colors):
	if "red_anime" in colors:
		return [0, 2]  # [color,mode]
	if "red" in colors:
		return [0, 1]
	if "yellow_anime" in colors:
		return [1, 2]
	if "yellow" in colors:
		return [1, 1]
	if "blue_anime" in colors:
		return [2, 2]
	if "blue" in colors:
		return [2, 1]
	
	#no match, all lights off
	return [0,0]

def set_leds(state):
	message = "Ampel is "
	if state[0] == 0:
		message += "red"
	elif state[0] == 1:
		message += "yellow"
	elif state[0] == 2:
		message += "blue"
	else:
		message = "Error: invalid color."
		return message

	if state[1] == 0:
		return message + "off."
	elif state[1] == 1:
		return message + "."
	elif state[1] == 2:
		return message + " and flashing."
	else:
		message = "Error: invalid mode."
		return message
	
	return message

	


if __name__ == "__main__":

	while True:
		url = 'http://localhost:8080/view/my_view/api/xml'
		colors = get_colors(url)
		state = get_state(colors)
	
		print set_leds(state)
		time.sleep(1)
	
