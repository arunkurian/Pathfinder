#!/usr/bin/env python
#
# Shape detction in static images with OpenCV
# by Arun Kurian

# Import the modules for the script
import cv2
import sys
import numpy as np
import math
import time

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

# Main

# Import shape image file
filename = 'images/shapes.png'
imgSRC = cv2.imread(filename)

# Setup Windows
cv2.namedWindow('Threshold Mask', cv2.WINDOW_NORMAL)
cv2.namedWindow('Canny imgEdges', cv2.WINDOW_NORMAL)
cv2.namedWindow('Shape Detection', cv2.WINDOW_NORMAL)

i = 0

while(1):

	start_time = time.time()

	k = cv2.waitKey(1)

	# ESC
	if k == 27:
		break

	# Setup destination image
	imgDst = imgSRC.copy()

	# Convert the image to HSV
	imgHSV = cv2.cvtColor(imgSRC, cv2.COLOR_BGR2HSV)

	# Set thresholds for HSV image
	lowerThreshHSV = np.array([40, 200, 200])
	upperThreshHSV = np.array([80, 255, 255])

	# Create mask from HSV thresholds
	imgMask = cv2.inRange(imgHSV, lowerThreshHSV, upperThreshHSV)
	cv2.imshow('Threshold Mask', imgMask)

	# Blur image before Canny edge detection
	blurKernelSize = 2
	img_blur = cv2.blur(imgMask, (blurKernelSize, blurKernelSize))
	
	# Use Canny edge detection to capture the imgEdges of the shapes
	cannyLowThreshold = 5
	cannyRatio = 3

	imgEdges = cv2.Canny(img_blur, cannyLowThreshold, cannyLowThreshold*cannyRatio, 5)

	# Display canny imgEdges
	cv2.imshow('Canny imgEdges', imgEdges)

	# Find the all contours 
	(contours, _) = cv2.findContours(imgEdges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	# For each contour set found in contours
	for contour in contours:

		# Approximate the contour with accuracy proportional to contour perimeter
		approxContour = cv2.approxPolyDP(contour, 
			cv2.arcLength(contour, True) * 0.02,
			True)

		# Skip small and non-convex contours
		if (abs(cv2.contourArea(contour)) < 100 ):
			continue

		# Draw the contour in green
		cv2.drawContours(imgDst, contour, -1, (255,255,255) , 2)

		# Calculate the contour area
		contourArea = cv2.contourArea(contour)

		# Calculate the number of vertices
		vertices = len(approxContour)

		# Evaluate minimum enclosing circle around contour
		(x,y),radius = cv2.minEnclosingCircle(contour)
		center = (int(x),int(y))
		radius = int(radius)
		cv2.circle(imgDst, center, radius, (0,0,255), 2)

		# Evaluate the minimum area rectangle around contour
		rect = cv2.minAreaRect(contour)
		box = cv2.cv.BoxPoints(rect)
		box = np.int0(box)

		boxArea = cv2.contourArea(box)
		cv2.drawContours(imgDst, [box], 0, (255,0,0), 2)	

		# Calculate area error between contour and min enclosing circle
		circleAreaError = abs(1 - (contourArea / (math.pi*radius*radius)))

		# Calculate area error between contour and min area rectangle
		rectAreaError = abs(1 - contourArea / boxArea)

		# Circle, if the enclosing circle area error below 20% (0.2)
		if (circleAreaError < 0.2):
			label = "CIRCLE"
			showLabel(imgDst, label, contour)

		# Rectangle, if the bounding rectangle area error below 15% (0.15)
		elif (rectAreaError < 0.15):
			label = "RECT"
			showLabel(imgDst, label, contour)

		# Triangle, if the bounding rectangle area error between 45% and 60% (0.45, 0.6)
		elif (rectAreaError > 0.45 and rectAreaError < 0.6):
			label = "TRI"
			showLabel(imgDst, label,contour)

		# Pentagon, if the bounding rectangle area error between 20% and 35% (0.2, 0.4) and vertices more than 4
		elif (rectAreaError < 0.35 and rectAreaError > 0.2 and vertices > 4):
			label = "PENTA"
			showLabel(imgDst, label,contour)

	# Display final image
	cv2.imshow('Shape Detection', imgDst)

	if (i == 10) :
		print("Loop time: ", time.time() - start_time)
		# cv2.imwrite('images/static_detect_beta_results.jpg', imgDst);
		i += 1
	else:
		i += 1

# Wait until a key is pressed, then exit
cv2.destroyAllWindows()