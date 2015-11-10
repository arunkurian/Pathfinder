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
	shapes = PathfinderCV('simple', 'linux')

	centroidThreshold = 5
	
	while not(shapes.detected):

		if abs(shapes.centroidError) > centroidThreshold:

			shapes.alignCamera()

		else:

			shapes.findShapes()

	print shapes.finalSet
	print time.time() - shapes.scriptStartTime	

except KeyboardInterrupt: 

	print 'Exited.'