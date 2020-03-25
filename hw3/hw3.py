'''
Copyright (c) 2020 Hao Da (Kevin) Dong
@file       hw3.py
@date       2020/02/20
@brief      Tracks green objects in a video feed and marks them with a bounding box
@license    This project is released under the BSD-3-Clause license.
'''

import numpy as np
import cv2
import datetime

# Set desired video resolution, framerate and logging offset
resolution = (1280,720)
fps = 15
logOffset = 3 		# Skip this number of datapoints while logging

# Set video codec and create video file
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, fps, resolution)

# Create video capture object and log file
videoCapture = cv2.VideoCapture(0)
f = open('hw3data.txt','a')

# Set HSV limits for thresholding
minHSV = np.array([40, 20, 25])
maxHSV = np.array([100, 180, 185])

# Number of desired datapoints to log
desiredDataPoints = 200
i = -logOffset

while(videoCapture.isOpened()):
	# Start timer for frame processing
	start = datetime.datetime.now()

	# Grab the current frame
	ret, image = videoCapture.read()
	
	# Convert to HSV and perform thresholding for green objects
	imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	greenMask = cv2.inRange(imageHSV, minHSV, maxHSV)

	# Draw bounding rectangle around green objects
	x, y, w, h = cv2.boundingRect(greenMask)
	cv2.rectangle(image, (x, y), (x+w, y+h), color=(0,0,255), thickness=2)

	# Display the frame
	cv2.imshow("Frame", cv2.resize(image, resolution))

	# Write frame to video file
	out.write(cv2.resize(image, resolution))
	
	# Stop timer
	stop = datetime.datetime.now()

	# Calculate and log processing time and framerate CMA (reject first few data points)
	if (i >= 0):
		processingTime = stop - start

		# Print and log processing time
		print("Processing Time: " + str(processingTime.total_seconds()))
		outstring = str(processingTime.total_seconds()) + '\n'
		f.write(outstring)
	
	# Exit if the user presses 'q'
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

	# If the desired number of data points has been collected, 
	# wait for final confirmation keypress and exit
	i = i + 1
	if (i == desiredDataPoints):
		cv2.waitKey(0)
		break

# Release video and file object handles
videoCapture.release()
out.release()
f.close()
print('Video and file handles closed')







