#!/usr/bin/env python
#
# Shape detction in video with OpenCV for Pathfinder robot
# by Arun Kurian

import cv2
import sys
import numpy as np
import time
import math

class PathfinderCV(object):

	# Initialize the PathfinderCV instance, prepare and start video capture
	def __init__(self, mode, camSource, camHue):

		self.mode = mode

		# Set script start time
		self.scriptStartTime = time.time()

		# Video capture
		self.cap = cv2.VideoCapture(camSource)

		# Setup camera with given hue
		self.prepCamera(camHue)

		# Arbitrary initial value for centroid error
		self.centroidError = 100

		# Initialize sets and count to 0
		self.shapeSets = []
		self.finalSet = []
		self.detected = 0

		if self.mode == 'debug':

			# Setup window
			cv2.namedWindow('Shape Detection', cv2.WINDOW_NORMAL)

	# Release capture destroy all windows, exit instance
	def __del__(self):

		# Release capture and exit
		self.cap.release()
		cv2.destroyAllWindows()

	# Prepare frame, find contours, identify bounding rectangle of the sign, and determine error 
	# between sign's vertical center and frame for tilt mechanism feedback
	def alignCamera(self):
		
		# Capture frame
		_, frame = self.cap.read()

		# self.findColor(frame)

		# Prepare frame with Gaussian blurring, HSV conversion, and Canny edge detection
		preppedFrame = self.prepFrame(frame)

		# Find the all contours 
		contours, hierarchy = cv2.findContours(preppedFrame.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

		# If contours exist
		if contours:

			# Define bounding box
			maxContour, maxRectangle, maxContourArea = self.detectBoundary(frame, contours)

			if not maxRectangle:
				
				self.flipColor()

		if self.mode == 'debug':

			# Display final image
			cv2.namedWindow('Shape Detection', cv2.WINDOW_NORMAL)
			cv2.imshow('Shape Detection', frame)
			cv2.moveWindow('Shape Detection', 360, 0)

		cv2.waitKey(1)

	# Prepare frame, find contours, identify bounding rectangle of the sign, determine centroid error,
	# find nested shapes within sign, and compute a final set of shapes after a sufficient number of iterations
	def findShapes(self):

		# Capture frame-by-frame
		_, frame = self.cap.read()

		# Prepare frame with Gaussian blurring, HSV conversion, and Canny edge detection
		preppedFrame = self.prepFrame(frame)
	
		# Find the all contours 
		contours, hierarchy = cv2.findContours(preppedFrame.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

		# If contours exist
		if contours:

			# Define bounding box
			maxContour, maxRectangle, maxContourArea = self.detectBoundary(frame, contours)

			# If a bounding box has been found
			if maxRectangle:

				# Identify all nested shapes
				self.detected = self.detectNestedShapes(frame, contours, hierarchy, maxContour, maxRectangle, maxContourArea)

			else: 

				self.centroidError = 100

		if self.mode == 'debug':

			# Display final image
			cv2.namedWindow('Shape Detection', cv2.WINDOW_NORMAL)
			cv2.imshow('Shape Detection', frame)
			cv2.moveWindow('Shape Detection', 360, 0)

		cv2.waitKey(1)

	# Setup the Playstation Eye camera with manual capture settings
	def prepCamera(self, camHue):

		# Set up camera frame size and rate for 320x240 at 30fps
		self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320)
		self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
		self.cap.set(cv2.cv.CV_CAP_PROP_FPS, 30)

		# Hue (Home)
		hue = camHue / 180.0

		# Brightness
		brightness = 0.0 / 256.0

		# Contrast
		contrast = 35.0 / 256.0

		# Saturation
		sat = 60.0 / 256.0

		# Gain
		gain = 15.0 / 64.0
		
		# Set up the camera color
		self.cap.set(cv2.cv.CV_CAP_PROP_HUE, hue)
		self.cap.set(cv2.cv.CV_CAP_PROP_BRIGHTNESS, brightness)
		self.cap.set(cv2.cv.CV_CAP_PROP_CONTRAST, contrast)
		self.cap.set(cv2.cv.CV_CAP_PROP_SATURATION, sat)
		self.cap.set(cv2.cv.CV_CAP_PROP_GAIN, gain)

		# Center of the frame
		self.frameCx = int(319.5 / 2.0)
		self.frameCy = int(239.5 / 2.0)

		# Focal lengths in pixels
		self.focal = (5.5933430628532710e+02) / 2.0

    # Take an individual frame and utilize Gaussian blurring, BGR-to-HSV color conversion, range thresholding, 
    # morphological transformation, and canny edge detection methods to prepare the frame for contour detection
	def prepFrame(self, frame):

		# Blur image before Canny edge detection
		blurKernelSize = 3
		imgBlur = cv2.GaussianBlur(frame, (blurKernelSize, blurKernelSize), 0)

		# Convert the image to HSV
		imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)

		# Create mask from HSV thresholds
		imgRangeMask = cv2.inRange(imgHSV, self.lowerHSV, self.upperHSV)

		# Morphological transformations
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
		imgMask = cv2.morphologyEx(imgRangeMask, cv2.MORPH_OPEN, kernel)

		# Use Canny edge detection to capture the edges of the shapes
		cannyLowThreshold = 5
		cannyRatio = 3

		preppedFrame = cv2.Canny(imgMask, cannyLowThreshold, cannyLowThreshold * cannyRatio, 5)

		if self.mode == 'debug':

			# Bitwise image
			imgBitwise = cv2.bitwise_and(frame, frame, mask = imgMask)

			# Display mask
			cv2.namedWindow('Threshold Mask', cv2.WINDOW_NORMAL)
			cv2.imshow('Threshold Mask', imgBitwise)
			cv2.moveWindow('Threshold Mask', 0, 0)

			cv2.namedWindow('Canny Edges', cv2.WINDOW_NORMAL)
			cv2.imshow('Canny Edges', preppedFrame)
			cv2.moveWindow('Canny Edges', 0, 300)

		# Return Canny edges to main
		return preppedFrame

	def flipColor(self):

		if (set(self.lowerHSV) == set(self.pinkLowerHSV)):

			self.lowerHSV = self.greenLowerHSV
			self.upperHSV = self.greenUpperHSV

			self.signColor = 1

		else: 

			self.lowerHSV = self.pinkLowerHSV
			self.upperHSV = self.pinkUpperHSV

			self.signColor = 0

	# Given a frame and contours found in that frame, determine the contour with the maximum area and 
	# an aspect ratio > 1.5, as this will likely correspond to the sign. Find the centroid of the contour 
	# and error between the frame center and contour center for tilt mechanism feedback
	def detectBoundary(self, frame, contours):

		if self.mode == 'debug':

			# Draw a dot at center of bounding rectangle
			cv2.circle(frame, (self.frameCx, self.frameCy), 5, (0,255,0), 2)

		# Set arbitrary contour area for initialization
		maxRectangle = []
		maxContour = []
		maxContourArea = 100

		# Identify max contour
		for contour in contours:

			# Calculate the contour area
			contourArea = abs(cv2.contourArea(contour))

			# Ignore small contours
			if (abs(contourArea) < 100):
				continue

			if (contourArea > maxContourArea):

				# Create a bounding rectangle
				maxRect = cv2.boundingRect(contour)

				if (maxRect[3] > 0):

					# Determine aspect ratio of bounding rectangle
					maxAspectRatio = maxRect[2] / maxRect[3]

					# Max contour must have AR > 1.5
					if (maxAspectRatio > 1.5):

						# Define new max contour
						maxContourArea = abs(contourArea)
						maxContour = contour
						maxRectangle = maxRect
		
		if maxContourArea > 100:

			(_, signCy), _ = cv2.minEnclosingCircle(maxContour)

			if self.mode == 'debug':

				# Draw bounding rectangle in white
				cv2.rectangle(frame, (maxRectangle[0], maxRectangle[1]), (maxRectangle[0] + maxRectangle[2], maxRectangle[1] + maxRectangle[3]), (255,255,255), 5)
		
				# Draw a dot at center of bounding rectangle
				cv2.circle(frame, (self.frameCx, int(signCy)), 2, (0,0,255), 2)
			
			# Find error in angles
			self.centroidError = int(math.degrees(math.atan2((signCy - self.frameCy) , self.focal)) / 6)

		return maxContour, maxRectangle, maxContourArea

	# Given contours and a maximum bounding contour,find all nested contours. For each nested contour, 
	# determine the number of vertices and area errors from bounding rectangles and circles. Utilize this 
	# data to identify the type of shape and iterate a global instance set. Once enough similar sets have 
	# been found, flag and return the final set of shapes in order of appearance (left of frame to right of frame)
	def detectNestedShapes(self, frame, contours, hierarchy, maxContour, maxRectangle, maxContourArea):

		iterShapes = []
		sortedSet = []

		nestedContours = []
		nestedContourAreas = []

		# Identify nested contours within max contour
		for i in range(0, len(contours) - 1):

			# Calculate the contour area
			contourArea = cv2.contourArea(contours[i])
			
			# Ignore small contours
			if (abs(contourArea) < 100):
				continue

			if (maxRectangle):

				((x,y),_) = cv2.minEnclosingCircle(contours[i])
				center = (int(x),int(y))

				# Use point-in-contour test to determine location of center relative to max contour
				inMaxContour = cv2.pointPolygonTest(maxContour, center, True)

				# If passes point-in-contour test, less than max contour area, and has no children
				if (inMaxContour > 0 and abs(contourArea) < 0.75 * maxContourArea and hierarchy[0][i][2] < 0) :

					nestedContours.append(contours[i])
					nestedContourAreas.append(abs(contourArea))

		# Calculate number of nested contours
		numContours = len(nestedContours)

		if (numContours == self.numIntersections):
		
			# Identify shape type for each of the nested contours
			for contour, contourArea in zip(nestedContours, nestedContourAreas):

				# Approximate the contour with accuracy proportional to contour perimeter
				approxContour = cv2.approxPolyDP(contour, 
					cv2.arcLength(contour, True) * 0.02,
					True)

				# Calculate the number of vertices
				vertices = len(approxContour)

				# Evaluate minimum enclosing circle around contour
				(x,y), radius = cv2.minEnclosingCircle(approxContour)
				center = (int(x),int(y))
				radius = int(radius)

				# Calculate enclosing circle area
				circleArea = 3.14 * radius * radius

				# Evaluate the minimum area rectangle around contour
				rect = cv2.minAreaRect(approxContour)
				rectBox = cv2.cv.BoxPoints(rect)
				rectBox = np.int0(rectBox)
				rectArea = cv2.contourArea(rectBox)

				# Calculate area error between contour and min enclosing circle
				circleAreaError = abs(1 - (contourArea / circleArea))

				# Calculate area error between contour and min area rectangle
				rectAreaError = abs(1 - (contourArea / rectArea))

				if self.mode == 'debug':

					# Draw bounding circle (green) and rectangle (blue) shapes
					cv2.circle(frame, center, radius, (0,255,0), 2)
					cv2.drawContours(frame, [rectBox], 0, (255,0,0), 2)

				# Find relative position in max rectangle
				relPos = round(float(center[0] - maxRectangle[0]) / maxRectangle[2], 2)

				# Circle, if the enclosing circle area error below 20% (0.2)
				if (circleAreaError < 0.25 and rectArea > circleArea):
					
					iterShapes.append((relPos, 'c'))
					if self.mode == 'debug':
						self.showLabel(frame, "CIRCLE", approxContour)

				# Square, if the bounding rectangle area error below 15% (0.15)
				elif (rectAreaError < 0.2 and circleArea > 1.5 * rectArea):
					
					iterShapes.append((relPos, 's'))
					if self.mode == 'debug':
						self.showLabel(frame, "SQR", approxContour)

				# Triangle, if the bounding rectangle area error between 45% and 60% (0.45, 0.6) and vertices < 10
				elif (rectAreaError > 0.4 and rectAreaError < 0.6 and vertices < 10):
					
					iterShapes.append((relPos, 't'))
					if self.mode == 'debug':
						self.showLabel(frame, "TRI", approxContour)

				# Triangle, if the bounding rectangle area error between 30% and 50% (0.3, 0.5) and vertices > 10
				elif (rectAreaError > 0.3 and rectAreaError < 0.5 and vertices > 10):

					iterShapes.append((relPos, 'x'))
					if self.mode == 'debug':
						self.showLabel(frame, "CROSS", approxContour)

			# Add iteration shape list to master shape list
			self.shapeSets.append(iterShapes)

			# Calculate final set once atleast 10 sets have been added
			if (len(self.shapeSets) > 10):

				idealShapeSets = 0
				shapeTypes = []
				finalTupleSet = []
				shapeSetList = [shape for shapeSet in self.shapeSets for shape in shapeSet]

				# Define ideal sets as those with length equal to max shape set size
				for shapeSet in self.shapeSets:

					if len(shapeSet) == self.numIntersections:
						idealShapeSets = idealShapeSets + 1

				# If count of the ideal sets > 10
				if (idealShapeSets > 10):

					# For each shape, add to shape types list
					for shape in shapeSetList:

						_, shapeType = shape

						if shapeType not in shapeTypes:
							shapeTypes.append(shapeType)

						if len(shapeTypes) == self.numIntersections:
							break

					# For each shape type, average positions and determine relative positions
					for shapeType in shapeTypes:

						relPositions = []

						for shape in shapeSetList:

							relPos, shapePosType = shape

							if shapePosType == shapeType:
								relPositions.append(relPos)

						# Average relative positions found for this shape type
						relPosition = sum(relPositions) / len(relPositions)

						# Find the integer position of the contour
						for num in range(0, self.numIntersections):

							lowerBound = num / float(self.numIntersections)
							upperBound = (num + 1) / float(self.numIntersections)

							# If relPos is higher than lower bound and less than upper bound, assign num
							if (relPosition > lowerBound and relPosition < upperBound):

								finalTupleSet.append((num + 1, shapeType))
								break

					if finalTupleSet:				

						finalTupleSet.sort(key=lambda tup: tup[0])
						self.finalSet = [shape[1] for shape in finalTupleSet]

						# Return detected variable as true
						return 1

	# Display labels in image frame window
	def showLabel(self, image, label, contour):

		# Get dimensions of the label
		labelText = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)

		# Find a bounding rectange for the contour
		labelRect = cv2.boundingRect(contour)

		# Calculate the center of the bounding rectangle from the positions and dimensions
		labelCenter = (labelRect[0] + ((labelRect[2] - labelText[0][0]) / 2), 
			labelRect[1] + ((labelRect[3] - labelText[0][1]) / 2))

		# Display a filled white rectangle at the center of the contour
		cv2.rectangle(image, labelCenter, (labelCenter[0] + labelText[0][0], labelCenter[1] - labelText[0][1]), ([255, 255, 255]), -1)

		# Display black text on top of the filled white rectangle
		cv2.putText(image, label, labelCenter, cv2.FONT_HERSHEY_SIMPLEX, 0.4, ([0, 0, 0]))
