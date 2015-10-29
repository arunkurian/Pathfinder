#!/usr/bin/env python
#
# ODROID-Arduino Main for Pathfinder
# by Arun Kurian

# Import the modules for the script
from pfcv import PathfinderCV
import sys
import serial
import time

LEDCommand = 'L'
ServoAddCommand = 'A'
ServoSubtractCommand = 'S'

try:

	arduino = serial.Serial('/dev/tty.usbmodemfa141', 9600, timeout=0.1)
	time.sleep(1)

	print 'Connected.'

	while True:

		time.sleep(2)

		shapes = PathfinderCV()

		centroidThreshold = 5
		
		while not(shapes.detected):

			if abs(shapes.centroidError) > centroidThreshold:

				shapes.alignCamera()
				if shapes.centroidError > 0:
					arduino.write(ServoAddCommand + str(abs(shapes.centroidError)))
				else:
					arduino.write(ServoSubtractCommand + str(abs(shapes.centroidError)))

			else:

				shapes.findShapes()

		for i in range(0, len(shapes.finalSet)):

			time.sleep(1)

			if (shapes.finalSet[i] == 'tri'):
				arduino.write(LEDCommand + '1')
			elif (shapes.finalSet[i] == 'circle'):
				arduino.write(LEDCommand + '2')
			elif (shapes.finalSet[i] == 'rect'):
				arduino.write(LEDCommand + '3')
			elif (shapes.finalSet[i] == 'cross'):
				arduino.write(LEDCommand + '4')

		# Clear shapes
		arduino.write(LEDCommand + '0')

except Exception, err:

	print 'Error.'
	print err

except KeyboardInterrupt: 

	print 'Exited.'