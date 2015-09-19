#!/usr/bin/env python
#
# Shape detction in static images with OpenCV
# by Arun Kurian
# 
# Adapted from "Detecting simple shapes in an image"
# http://opencv-code.com/tutorials/detecting-simple-shapes-in-an-image/

# Import the modules for the script
import cv2
import sys
import numpy as np
import math

# Helper function to determine the cosine of the angle
# between two vectors originating at the same point
def getCosine(point1, point2, point0):

	# Calculate direction vectors from contour points
	vector1 = [a - b for a, b in zip(point1, point0)]
	vector2 = [a - b for a, b in zip(point2, point0)]

	# Unitize direction vectors with norm
	unitVector1 = vector1 / np.linalg.norm(vector1)
	unitVector2 = vector2 / np.linalg.norm(vector2)

	# Convert numpy arrays to lists
	unitVector1 = np.array(unitVector1).tolist()
	unitVector2 = np.array(unitVector2).tolist()

	# Use the dot product of the two unit vectors for cosine in between
	cosine = sum((a * b) for a, b in zip(unitVector1[0], unitVector2[0]))

	return cosine

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
filename = 'images/simple-shapes.png'
img_src = cv2.imread(filename)
img_dst = img_src.copy()

# Convert the image to grayscale
img_gray = cv2.cvtColor(img_src, cv2.COLOR_BGR2GRAY)

# Blur image before Canny edge detection
img_blur = cv2.blur(img_gray, (3,3))

# Use Canny edge detection to capture the edges of the shapes
edges = cv2.Canny(img_blur, 0, 50, 5)

# Find the all contours 
(contours, _) = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# For each contour set found in contours
for contour in contours:

	# Approximate the contour with accuracy proportional to contour perimeter
	approxContour = cv2.approxPolyDP(contour, 
		cv2.arcLength(contour, True) * 0.02,
		True)

	# Skip small and non-convex contours
	if (abs(cv2.contourArea(contour)) < 100 or not(cv2.isContourConvex(approxContour))):
		continue

	vertices = len(approxContour)

	# Triangle, if number of vertices is 3
	if (vertices == 3):
		showLabel(img_dst, "TRI", contour)

	# If number of vertices is >= 3 and <= 6
	elif (vertices >= 4 and vertices <= 6):
		
		cosines = []

		# Determine the cosine of all angles in shape
		for j in range(2, vertices+1):
			cosines.append(getCosine(approxContour[j%vertices], approxContour[j-2], approxContour[j-1]))

		minCos = min(cosines)
		maxCos = max(cosines)

		# Square, if 4 vertices and all angles between 70 and 95 degrees
		if (vertices == 4 and minCos >= -0.1 and maxCos <= 0.3):
			showLabel(img_dst, "RECT", contour)

		# Pentagon, if 5 vertices and all angles between 105 and 110 degrees
		elif ( vertices == 5 and minCos >= -0.34 and maxCos <= -0.27):
			showLabel(img_dst, "PENTA", contour)

		# Hexagon, if 6 vertices and all angles between 117 and 123 degrees
		elif ( vertices == 6 and minCos >= -0.55 and maxCos <= -0.45):
			showLabel(img_dst, "HEXA", contour)

	# If number of vertices not in [3, 6]
	else:

		# Calculate the contour area
		area = cv2.contourArea(contour)

		# Determine the circle's bounding rectangle
		circleRect = cv2.boundingRect(contour)

		# Calculate errors in the radius and area
		radiusError = abs(1 - (circleRect[2] / circleRect[3]))
		areaError = abs(1 - (area / (math.pi*circleRect[2]*circleRect[2]/4)))

		# Circle, if the errors area below 20% (0.2)
		if (radiusError <= 0.2 and areaError <= 0.2):
			showLabel(img_dst, "CIRCLE", contour)

# Display final image
cv2.namedWindow('Shape Detection', cv2.WINDOW_NORMAL)
cv2.imshow('Shape Detection', img_dst)

# Wait until a key is pressed, then exit
cv2.waitKey(0)
cv2.destroyAllWindows()