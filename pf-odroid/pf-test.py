#!/usr/bin/env python
#
# ODROID-Arduino Test Program for Pathfinder
# by Arun Kurian

# Import the modules for the script
from pfcv import PathfinderCV
import time

try:

	# Takes two modes: debug, simple
	# Takes two OSs: mac, linux
	shapes = PathfinderCV('debug', 'linux')

	centroidThreshold = 5
	
	while(True):

		shapes.findShapes()

		if shapes.shapeSets:
			if not shapes.finalSet:
				print shapes.shapeSets[-1]	
			else:
				print shapes.finalSet

except KeyboardInterrupt: 

	print 'Exited.'