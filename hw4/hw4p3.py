'''
Copyright (c) 2020 Hao Da (Kevin) Dong
@file       hw4p3.py
@date       2020/02/26
@brief      Tracks a green arrow through a video feed and displays its orientation
@license    This project is released under the BSD-3-Clause license.
'''

import numpy as np
import cv2
import datetime


'''
@brief      Checks if an angle can be considered 90 degrees
@param      Angle
@return     True/False
'''
def checkIf90(angle):
	return 85 <= angle <= 95


'''
@brief      Detects an arrow in an image and finds the corners and their angles
@param      image				NumPy array of a BGR image
@return     defaultCornerSet	NumPy array of arrow corner coordinates
@return		cornerAngles		NumPy array of arrow corner angles
'''
def getArrowCorners(image):
	# Set HSV limits for thresholding
	minHSV = np.array([40, 130, 100])
	maxHSV = np.array([100, 255, 255])

	# Apply Gaussian blur to remove noise
	image = cv2.GaussianBlur(image, (5,5), 0)

	# Convert image to HSV and apply thresholding to isolate green area (arrow)
	imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	greenMask = cv2.inRange(imageHSV, minHSV, maxHSV)

	# Extract the contour of the arrow
	contours, hierarchy = \
				cv2.findContours(greenMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


	defaultCornerSet = np.array([])
	cornerAngles = np.array([])

	if (np.shape(contours)[0] != 0):
		# If an arrow is detected, reduce its contour to the minimum number of vertices (7)
		epsilon = 1
		cornerSet = cv2.approxPolyDP(contours[0], epsilon, closed=True)

		# Continue to increase the error tolerance until the approximation produces 7 points
		while (True):
			# If an invalid contour is detected, do not process it further
			if (np.shape(cornerSet)[0] <= 7):
				break

			epsilon = epsilon + 1
			cornerSet = cv2.approxPolyDP(contours[0], epsilon, closed=True)

		# If a valid contour is found, calculate and return an array of its corner angles
		if (np.shape(cornerSet)[0] == 7):
			numCorners = np.shape(cornerSet)[0]
			cornerAngles = np.zeros(numCorners)

			defaultCornerSet = cornerSet

			for i in range(numCorners):
				# Ensure that loop indices wrap around the array
				prev_i = i-1
				if (prev_i < 0):
					prev_i = numCorners-1
				
				next_i = i+1
				if (next_i > numCorners-1):
					next_i = 0

				# Determine the segment vectors at the corner
				segment1 = cornerSet[i] - cornerSet[prev_i] 
				segment2 = cornerSet[i] - cornerSet[next_i]

				# Compute the angle between the vectors
				theta = np.arccos(np.dot(segment1, segment2.T) / 
								(np.linalg.norm(segment1)*np.linalg.norm(segment2)))

				cornerAngles[i] = np.degrees(theta)

	return defaultCornerSet, cornerAngles


'''
@brief      Determines an arrow's orientation angle from its corners and corner angles
@param    	cornerSet		NumPy array of arrow corner coordinates
@param		cornerAngles	NumPy array of arrow corner angles
@return     arrowAngle		Arrow's angle between -180 to 180 degrees
'''
def getArrowAngle(cornerSet, cornerAngles):
	numCorners = np.shape(cornerSet)[0]

	arrowAngle = float('nan')

	for i in range(numCorners):
		# Ensure that loop indices wrap around the array
		prev_i = i-1
		if (prev_i < 0):
			prev_i = numCorners-1
		
		next_i = i+1
		if (next_i > numCorners-1):
			next_i = 0

		# If a corner is 90 deg and its neighbouring corners are not, then it is the arrowhead
		if (checkIf90(cornerAngles[i]) == True):
			if (checkIf90(cornerAngles[prev_i]) == False and checkIf90(cornerAngles[next_i]) == False):
				# Get arrowhead and neighbouring corner coordinates in the standard xy frame
				rightVertex = np.array([[cornerSet[prev_i, 0, 0], -cornerSet[prev_i, 0, 1]]])
				centerVertex = np.array([[cornerSet[i, 0, 0], -cornerSet[i, 0, 1]]])
				leftVertex = np.array([[cornerSet[next_i, 0, 0], -cornerSet[next_i, 0, 1]]])
				
				# Computer arrowhead side vectors
				segment1 = centerVertex - leftVertex
				segment2 = centerVertex - rightVertex

				# Compute arrow vector as the resultant of the segments and find the angle
				arrowVector = segment1 + segment2
				arrowAngle = np.degrees(np.arctan2(arrowVector[0,1], arrowVector[0,0]))
	
	return arrowAngle

				

if __name__ == '__main__':
	# Set desired video resolution, framerate and logging offset
	resolution = (1280,720)
	fps = 15
	logOffset = 3 		# Skip this number of datapoints while logging

	# Create video capture object and log file
	videoObject = cv2.VideoCapture(0)
	f = open('hw4data.txt','a')

	# Set video codec and create video file
	fourcc = cv2.VideoWriter_fourcc(*'mp4v')
	out = cv2.VideoWriter('output.mp4', fourcc, fps, resolution)


	# Number of desired datapoints to log
	desiredDataPoints = 200
	i = -logOffset

	while(videoObject.isOpened()):
		# Start timer for frame processing
		start = datetime.datetime.now()

		# Grab the current frame
		ret, frame = videoObject.read()

		if ret == True:
			# Get arrow corners and their angles and check validity
			cornerSet, cornerAngles = getArrowCorners(frame)

			if (np.shape(cornerSet)[0] == 0):
				continue
			
			# Get arrow angle and check validity
			arrowAngle = getArrowAngle(cornerSet, cornerAngles)

			if (np.isnan(arrowAngle) == True):
				continue
			
			# Circle arrow corners in the frame
			for corner in cornerSet:
				cv2.circle(frame, tuple(corner.tolist()[0]), 5, color=(0,0,255), thickness=3)

			font = cv2.FONT_HERSHEY_COMPLEX_SMALL
			direction = 'No Arrow'

			# Determine arrow direction
			if (45 < arrowAngle < 135):
				direction = 'UP'
			elif (-45 < arrowAngle < 45):
				direction = 'RIGHT'
			elif (-135 < arrowAngle < -45):
				direction = 'DOWN'
			else:
				direction = 'LEFT'
			
			# Overlay arrow angle and direction on frame
			cv2.putText(frame, direction, (10,30), font, 1, color=(0,255,0), thickness=2)
			cv2.putText(frame, 'Angle: ' + str(round(arrowAngle)), \
				(10,80), font, 1, color=(0,255,0), thickness=2)

			# Display the frame
			cv2.imshow("Frame", cv2.resize(frame, resolution))

			# Write frame to video file
			out.write(cv2.resize(frame, resolution))
			
			# Calculate and log processing time (reject first few data points)
			if (i >= 0):
				stop = datetime.datetime.now()
				processingTime = stop - start
				outstring = str(processingTime.total_seconds()) + '\n'
				f.write(outstring)
				print("Processing Time: " + str(processingTime.total_seconds()))

			i = i + 1

			# If the desired number of data points has been collected, 
			# wait for final confirmation keypress and exit
			if (i == desiredDataPoints):
				cv2.waitKey(0)
				break
			
			# Exit if the user presses 'q'
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		
		else:
			cv2.waitKey(0)
			break


	# Release all video and file object handles
	videoObject.release()
	out.release()
	f.close()
	print('Video and file handles closed')








