#!/usr/bin/env python
#
# Shape detction in video with OpenCV
# by Arun Kurian

# Import the modules for the script
import cv2
import sys
import numpy as np
import math
import time
from operator import itemgetter

# Helper function to display labels in image
def showLabel(image, label, contour):

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

# Helper function to get region of nested contour
def getContourRegion(maxRectangle, center, numContours):

	# Get relative X position of contour in max rectangle
	relPos = float(center[0] - maxRectangle[0]) / maxRectangle[2]

	# Find the integer position of the contour
	for num in range(1, numContours):

		lowerBound = (num - 1) / float(numContours)
		upperBound = num / float(numContours)

		# If relPos is higher than lower bound and less than upper bound, assign num
		if (relPos > lowerBound and relPos < upperBound):

			return num - 1

	# Else, return num of contours (boneyard)
	return numContours

# Main

# Import shape image file
cap = cv2.VideoCapture(0)

scriptStartTime = time.time()

# Setup Windows
# cv2.namedWindow('Threshold Mask', cv2.WINDOW_NORMAL)
cv2.namedWindow('Shape Detection', cv2.WINDOW_NORMAL)

count = 0
shapeSets = []
shapeSizes = []
sortedSet = []
finalSet = []

while(True):

	loopStartTime = time.time()

	k = cv2.waitKey(1)

	# ESC
	if k == 27:
		break

	# Capture frame-by-frame
	(_, frame) = cap.read()

	# Blur image before Canny edge detection
	blurKernelSize = 3
	imgBlur = cv2.GaussianBlur(frame, (blurKernelSize, blurKernelSize), 0)

	# Convert the image to HSV
	imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)

	# Set thresholds for HSV image
	lowerThreshHSV = np.array([150, 100, 100])
	upperThreshHSV = np.array([180, 255, 255])

	# Create mask from HSV thresholds
	imgRangeMask = cv2.inRange(imgHSV, lowerThreshHSV, upperThreshHSV)

	# Morphological transformations
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
	imgMask = cv2.morphologyEx(imgRangeMask, cv2.MORPH_OPEN, kernel)
	
	# Display mask
	# cv2.imshow('Threshold Mask', imgMask)

	# Use Canny edge detection to capture the imgEdges of the shapes
	cannyLowThreshold = 5
	cannyRatio = 3

	imgEdges = cv2.Canny(imgMask, cannyLowThreshold, cannyLowThreshold * cannyRatio, 5)

	# Find the all contours 
	(contours, hierarchy) = cv2.findContours(imgEdges.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

	# Set arbitrary contour area for initialization
	maxContourArea = 100
	nestedContours = []

	# Identify max contour
	for contour in contours: 

		# Approximate the contour with accuracy proportional to contour perimeter
		approxContour = cv2.approxPolyDP(contour, 
			cv2.arcLength(contour, True) * 0.02,
			True)

		# Calculate the contour area
		contourArea = abs(cv2.contourArea(approxContour))

		# Ignore small contours
		if (abs(contourArea) < 100):
			continue

		if (contourArea > maxContourArea):

			# Create a bounding rectangle
			maxRect = cv2.boundingRect(approxContour)

			maxRectWidth = maxRect[2]
			maxRectHeight = maxRect[3]

			if (maxRectHeight > 0):

				# Determine aspect ratio of bounding rectangle
				maxAspectRatio = maxRectWidth / maxRectHeight

				# Max contour must have AR > 1.5
				if (maxAspectRatio > 1.5):

					# Define new max contour
					maxContourArea = abs(contourArea)
					maxContour = approxContour
					maxRectangle = maxRect

					# Draw bounding rectangle in white
					cv2.rectangle(frame, (maxRect[0], maxRect[1]), (maxRect[0] + maxRectWidth, maxRect[1] + maxRectHeight), (255,255,255), 5)

	# Identify nested contours within max contour
	for i in range(0, len(contours) - 1):

		# Approximate the contour with accuracy proportional to contour perimeter
		approxContour = cv2.approxPolyDP(contours[i], 
			cv2.arcLength(contours[i], True) * 0.02,
			True)

		# Calculate the contour area
		contourArea = cv2.contourArea(approxContour)
		
		# Ignore small contours
		if (abs(contourArea) < 100):
			continue

		# Be sure max contour is first founded
		if (maxContourArea > 100):

			((x,y),_) = cv2.minEnclosingCircle(approxContour)
			center = (int(x),int(y))

			# Use point-in-contour test to determine location of center relative to max contour
			inMaxContour = cv2.pointPolygonTest(maxContour, center, True)

			# If passes point-in-contour test, less than max contour area, and has no children
			if (inMaxContour > 0 and abs(contourArea) < 0.75 * maxContourArea and hierarchy[0][i][2] < 0) :

				nestedContours.append(approxContour)

	# Calculate number of nested contours
	numContours = len(nestedContours)
	
	iterShapes = []

	# Identify shape type for each of the nested contours
	for contour in nestedContours:

		# Calculate the contour area
		contourArea = cv2.contourArea(contour)

		# Calculate the number of vertices
		vertices = len(contour)

		# Evaluate minimum enclosing circle around contour
		(x,y),radius = cv2.minEnclosingCircle(contour)
		center = (int(x),int(y))
		radius = int(radius)

		# Calculate enclosing circle area
		circleArea = math.pi * radius * radius

		# Evaluate the minimum area rectangle around contour
		rect = cv2.minAreaRect(contour)
		rectBox = cv2.cv.BoxPoints(rect)
		rectBox = np.int0(rectBox)
		rectArea = cv2.contourArea(rectBox)

		# Calculate area error between contour and min enclosing circle
		circleAreaError = abs(1 - (contourArea / circleArea))

		# Calculate area error between contour and min area rectangle
		rectAreaError = abs(1 - (contourArea / rectArea))

		# Draw bounding circle (green) and rectangle (blue) shapes
		cv2.circle(frame, center, radius, (0,255,0), 2)
		cv2.drawContours(frame, [rectBox], 0, (255,0,0), 2)

		# Find relative position in max rectangle
		relPos = getContourRegion(maxRectangle, center, numContours)

		# Circle, if the enclosing circle area error below 20% (0.2)
		if (circleAreaError < 0.2 and rectArea > circleArea):
			showLabel(frame, "CIRCLE", contour)
			iterShapes.insert(relPos, 'circle')

		# Rectangle, if the bounding rectangle area error below 15% (0.15)
		elif (rectAreaError < 0.15 and circleArea > 1.5 * rectArea):
			showLabel(frame, "RECT", contour)
			iterShapes.insert(relPos, 'rect')

		# Triangle, if the bounding rectangle area error between 45% and 60% (0.45, 0.6) and vertices < 5
		elif (rectAreaError > 0.45 and rectAreaError < 0.6 and vertices < 5):
			showLabel(frame, "TRI", contour)
			iterShapes.insert(relPos, 'tri')

		# Triangle, if the bounding rectangle area error between 30% and 50% (0.3, 0.5) and vertices > 10
		elif (rectAreaError > 0.3 and rectAreaError < 0.5 and vertices > 10):
			showLabel(frame, "CROSS", contour)
			iterShapes.insert(relPos, 'cross')

	# If number of contours > 0 and equal to shape list
	if (numContours > 0 and len(iterShapes) == numContours):

		# Add iteration shape list to master shape list
		shapeSets.append(iterShapes)

		# Add size of shape list to shape size list
		shapeSizes.append(len(iterShapes))

	# If number of iteration shape lists > 10
	if (len(shapeSizes) > 10):
		
		# Find the mode of the shape size list
		shapeSize = max(set(shapeSizes), key = shapeSizes.count)

		# If count of the mode value > 10
		if (shapeSizes.count(shapeSize) > 10):

			# For each shape set, add to sorted list if size equals mode shape size
			for shapeSet in shapeSets:

				if (len(shapeSet) == shapeSize):

					sortedSet.append(shapeSet)
					print shapeSet

			# For each column in sorted set, find mode shape and assign to final set
			for i in range(shapeSize):

				sortedCol = [row[i] for row in sortedSet];
				modeCol = max(set(sortedCol), key = sortedCol.count)
				
				finalSet.insert(i, modeCol)

			# Exit after all shapes have been found
			break

	# Display final image
	cv2.imshow('Shape Detection', frame)

	count += 1

	if count == 10:
		cv2.imwrite("./result.png", frame)


print '\n'
print "Final Set: " + ", ".join(finalSet).upper()
print "Loop Time: " + str((time.time() - scriptStartTime) / count)
print "Total Time: " + str(time.time() - scriptStartTime)

# Wait until a key is pressed, then release capture and exit
cap.release()
cv2.destroyAllWindows()