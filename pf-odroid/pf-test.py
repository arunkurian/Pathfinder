#!/usr/bin/env python
#
# ODROID-Arduino Test Program for Pathfinder
# by Arun Kurian

# Import the modules for the script
from pfcv import PathfinderCV
import numpy as np
import time

def setColorRanges(setting):

	# If Home
	if setting == 'home':

		# Pink
		pinkLowerHSV = np.array([135, 75, 50])
		pinkUpperHSV = np.array([170, 255, 255])
		
		# Green
		greenLowerHSV = np.array([20, 75, 50])
		greenUpperHSV = np.array([60, 255, 255])

	# If Lab
	elif setting == 'lab':

		# Pink
		pinkLowerHSV = np.array([135, 50, 0])
		pinkUpperHSV = np.array([179, 255, 255])

		# Green
		greenLowerHSV = np.array([20, 75, 50])
		greenUpperHSV = np.array([60, 255, 255])

	return (pinkLowerHSV, pinkUpperHSV, greenLowerHSV, greenUpperHSV)

try:

	# Mode: debug, simple
	# Camera source: Linux VM - 1, ODROID - 0
	# Camera hue: Home - 45.0, Lab - 125.0
	shapes = PathfinderCV('debug', 1, 45.0)

	shapes.numIntersections = 3

	(shapes.pinkLowerHSV, shapes.pinkUpperHSV, shapes.greenLowerHSV, shapes.greenUpperHSV) = setColorRanges('home')
	shapes.lowerHSV = shapes.pinkLowerHSV
	shapes.upperHSV = shapes.pinkUpperHSV

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