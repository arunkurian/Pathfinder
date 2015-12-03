#!/usr/bin/env python
#
# ODROID-Arduino Test Program for Pathfinder
# by Arun Kurian

# Import the modules for the script
from pfcv import PathfinderCV
import numpy as np
import time

try:

	# Camera source: Linux VM - 1, ODROID - 0
	# Camera hue: Home - 45.0, Lab - 125.0
	shapes = PathfinderCV(1, 45.0)

	shapes.numIntersections = 3

	shapes.signColor = 0

	centroidThreshold = 1
	shapeSets = 0

	while(True):

		if (abs(shapes.centroidError) > centroidThreshold):
			shapes.alignCamera()
		else:
			shapes.findShapes()

		if shapes.shapeSets and len(shapes.shapeSets) > shapeSets:

			print shapes.signColor
			shapeSets = len(shapes.shapeSets)

			if not shapes.finalSet:
				print shapes.shapeSets[-1]
			else:
				print shapes.finalSet

except KeyboardInterrupt: 

	print 'Exited.'