#!/usr/bin/env python
#
# ODROID-Arduino Main for Pathfinder
# by Arun Kurian

# Import the modules for the script
from pfcv import PathfinderCV
import sys
import serial
import time

RobotTurnCommand = 'T'
ServoAddCommand = 'A'
ServoSubtractCommand = 'S'
QuitCommand = 'Q'

try:

	arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

	time.sleep(1)
	arduino.write('1')
	
	time.sleep(1)
	connected = arduino.read(1)

	if (connected == '1'): 

		print 'Connected to Arduino.'

		while True:

			incomingCmd = arduino.read(4)
			print incomingCmd

			if (incomingCmd == 'pfcv'):

				desiredShape = chr(int(arduino.read(3)))
				print desiredShape

				# Takes two modes: debug, simple
				# Takes two OSs: mac, linux
				shapes = PathfinderCV('debug', 'linux')

				centroidThreshold = 0.5
				
				while not(shapes.detected):

					if (abs(shapes.centroidError) > centroidThreshold):

						shapes.alignCamera()
						if shapes.centroidError > 0:
							arduino.write(ServoAddCommand + str(abs(shapes.centroidError)))
						else:
							arduino.write(ServoSubtractCommand + str(abs(shapes.centroidError)))

					else:

						shapes.findShapes()

				print "Final Set: " + ", ".join(shapes.finalSet).upper()

				if (desiredShape in shapes.finalSet):
					shapeIndex = shapes.finalSet.index(desiredShape) + 1
				else:
					shapeIndex = 0

				print shapeIndex
				arduino.write(RobotTurnCommand + str(shapeIndex))
				time.sleep(1)
				arduino.write(QuitCommand)
				del shapes

			if (incomingCmd == 'exit'):

				break

	arduino.close()
	sys.exit(1)

except Exception, err:

	print 'Error.'
	print err
	sys.exit(1)

except KeyboardInterrupt: 

	print 'Exited.'
	sys.exit(1)