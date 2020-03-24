'''
Copyright (c) 2020 Hao Da (Kevin) Dong
@file       hw4.py
@date       2020/02/26
@brief      TBD
@license    This project is released under the BSD-3-Clause license.
'''

#from picamera import PiCamera
import numpy as np
import cv2
import imutils
import time
import datetime

#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('output.avi',fourcc, 2, (720,480))

#f = open('hw3data.txt','a')

'''
# Initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (720,480)
camera.framerate = 60
'''

videoObject = cv2.VideoCapture(0)

#fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#out = cv2.VideoWriter('CubeOverlayOutput.mp4', fourcc, 30, (720, 480))

desiredHSV = np.array([70, 210, 180])
thresholds = np.array([30, 80, 80])
  
minHSV = np.clip(desiredHSV - thresholds, 0, 255)
maxHSV = np.clip(desiredHSV + thresholds, 0, 255)


# Keep looping
#desiredDataPoints = 150
#i = 0

while(True):
	ret, frame = videoObject.read()

	if ret == True:
		image = frame

		# Apply median blur to remove pixel noise
		image = cv2.GaussianBlur(image, (5,5), 0)

		imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		greenMask = cv2.inRange(imageHSV, minHSV, maxHSV)

		contours, hierarchy = \
					cv2.findContours(greenMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		if (np.shape(contours)[0] != 0):
			epsilon = 1
			cornerSet = cv2.approxPolyDP(contours[0], epsilon, closed=True)

			# Continue to increase the error tolerance until the approximation produces four points
			while (True):
				if (np.shape(cornerSet)[0] <= 7):
					break

				epsilon = epsilon + 1
				cornerSet = cv2.approxPolyDP(contours[0], epsilon, closed=True)

			#print(cornerSet)

			if (np.shape(cornerSet)[0] == 7):

				numCorners = np.shape(cornerSet)[0]
				cornerAngles = np.zeros(numCorners)
				for i in range(numCorners):
					prev_i = i-1
					if (prev_i < 0):
						prev_i = numCorners-1
					
					next_i = i+1
					if (next_i > numCorners-1):
						next_i = 0

					segment1 = cornerSet[i] - cornerSet[prev_i] 
					segment2 = cornerSet[i] - cornerSet[next_i]

					theta = np.arccos(np.dot(segment1, segment2.T) / 
									(np.linalg.norm(segment1)*np.linalg.norm(segment2)))
					cornerAngles[i] = np.degrees(theta)

				#print(cornerAngles)

				def checkIf90(angle):
					return 85 <= angle <= 95

				arrowVector = np.array([])
				arrowAngle = []
				for i in range(numCorners):
					prev_i = i-1
					if (prev_i < 0):
						prev_i = numCorners-1
					
					next_i = i+1
					if (next_i > numCorners-1):
						next_i = 0

					if (checkIf90(cornerAngles[i]) == True):
						if (checkIf90(cornerAngles[prev_i]) == False and checkIf90(cornerAngles[next_i]) == False):

							rightVertex = np.array([[cornerSet[prev_i, 0, 0], -cornerSet[prev_i, 0, 1]]])
							centerVertex = np.array([[cornerSet[i, 0, 0], -cornerSet[i, 0, 1]]])
							leftVertex = np.array([[cornerSet[next_i, 0, 0], -cornerSet[next_i, 0, 1]]])
							
							segment1 = centerVertex - leftVertex
							segment2 = centerVertex - rightVertex

							arrowVector = segment1 + segment2
							arrowAngle = np.degrees(np.arctan2(arrowVector[0,1], arrowVector[0,0]))

							for corner in cornerSet:
								cv2.circle(image, tuple(corner.tolist()[0]), 5, color=(0,0,255), thickness=3)

							font = cv2.FONT_HERSHEY_COMPLEX_SMALL
							direction = 'No Arrow'

							if (45 < arrowAngle < 135):
								direction = 'UP'
							elif (-45 < arrowAngle < 45):
								direction = 'RIGHT'
							elif (-135 < arrowAngle < -45):
								direction = 'DOWN'
							else:
								direction = 'LEFT'

							cv2.putText(image, direction, (10,30), font, 1, color=(0,0,255))
							cv2.putText(image, 'Angle: ' + str(round(arrowAngle)), \
								(10,80), font, 1, color=(0,0,255))

				
		# greenMaskBGR = cv2.cvtColor(greenMask, cv2.COLOR_GRAY2BGR)
		cv2.imshow('greenMask', image)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	
	else:
		cv2.waitKey(0)
		break

#cv2.imshow('greenMask', greenMaskBGR)
cv2.waitKey(0)

videoObject.release()
#maskGray = cv2.cvtColor(greenMask, cv2.COLOR_GRAY2BGR)

#filteredImage = cv2.bitwise_and(image, image, mask = greenMask)








