'''
Copyright (c) 2020 Hao Da (Kevin) Dong
@file       VideoStream.py
@date       2020/03/20
@brief      Collects framerate data on the RPi camera while using the imutils VideoStream
			class multithreading methods
@license    This project is released under the BSD-3-Clause license.
'''

import numpy as np
import cv2
from imutils.video import VideoStream
import datetime
import time


# Set desired video resolution, framerate and logging offset
resolution = (1280,720)
fps = 30
logOffset = 3 		# Skip this number of datapoints while logging

# Create MP4 video file at 720p 30fps
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, fps, resolution)

# Create file to log data
f = open('VideoStreamTestData.txt','a')

# Create video stream object
vs = VideoStream(src=0, usePiCamera=False, resolution=resolution, framerate=fps).start()
time.sleep(1)

minHSV = np.array([40, 20, 25])
maxHSV = np.array([100, 180, 185])

# Framerate cumulative moving average
cmaFramerate = 0

# Number of desired datapoints to log
desiredDataPoints = 200
i = -logOffset

while(True):
	# Start timer for frame processing
	start = datetime.datetime.now()

	# Grab the current frame
	image = vs.read()
	
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
		framerate = 1 / processingTime.total_seconds()
		cmaFramerate = cmaFramerate + (framerate - cmaFramerate)/(i+1)

		# Print and log processing time and framerate CMA
		print("Frame number: " + str(i))
		print("Processing Time: " + str(processingTime.total_seconds()))
		print("Framerate CMA: " + str(cmaFramerate) + '\n')
		outstring = str(processingTime.total_seconds()) + '\t' + str(cmaFramerate) + '\n'
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
vs.stop()
out.release()
f.close()
print('Video and file handles closed')









