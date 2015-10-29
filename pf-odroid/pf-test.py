#!/usr/bin/env python
#
# ODROID-Arduino Test Program for Pathfinder
# by Arun Kurian

# Import the modules for the script
from pfcv import PathfinderCV

try:

	shapes = PathfinderCV()

	centroidThreshold = 5

	while (True):

		shapes.findShapes()
	
	# print '\n'
	# print "Final Set: " + ", ".join(self.finalSet).upper()
	# print "Total Time: " + str(time.time() - self.scriptStartTime)

except KeyboardInterrupt: 

	print 'Exited.'