'''
Copyright (c) 2020 Hao Da (Kevin) Dong, Krithika Govindaraj
@file       imuTeleoperation.py
@date       2020/05/01
@brief      Teleoperation of the Baron, using both encoders and IMU for localization
@license    This project is released under the BSD-3-Clause license.
'''

import RPi.GPIO as gpio
import serial
import time
import numpy as np
#import matplotlib.pyplot as plt
import math
import cv2
#from scipy.spatial import distance as dist
import imutils
############################## FILE TO STORE COORDINATES ##################
#f = open("demofile.txt", "a")
############################## VIDEO SETTINGS #############################

# Set desired video resolution, framerate and logging offset
resolution = (1280,720)
fps = 15
logOffset = 3 		# Skip this number of datapoints while logging

# Set video codec and create video file
#fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#out = cv2.VideoWriter('output.mp4', fourcc, fps, resolution)

# Create video capture object and log file
videoCapture = cv2.VideoCapture(0)

# Set HSV limits for thresholding
minHSV = np.array([106, 152, 0])
maxHSV = np.array([117, 255, 255])

################################# CLAW OPERATION ##############################
# Claw parameters
clawPin = 36
closePWM = 5.5
openPWM = 9

# Setup RPi GPIO pins
gpio.cleanup()
gpio.setmode(gpio.BOARD)
gpio.setup(clawPin, gpio.OUT)
claw = gpio.PWM(clawPin, 50)		# Set PWM to 50 Hz

# Start claw in closed position
claw.start(closePWM)

# Create claw PWM duty cycle sequence
openSequence = np.arange(closePWM, openPWM+0.5, 0.5)
closeSequence = np.flip(openSequence)
fullSequence = np.hstack((openSequence, closeSequence))
################################## TRAJECTORY ########################################
X = [0] # X coords
Y = [0]	# Y coords
# forwardCoord = "Y"  initially if we move forward we are moving in Y
# backCoord = "Y"
# rightCoord = "X"   # initially if we move right we are moving in X
# leftCoord = "X"	   # initially if we move left we are moving in X
currentAngle = 90      # facing north
################################## IMU SERIAL CONNECTION ############################
# Create serial connection
ser = serial.Serial('/dev/ttyUSB0', 9600)
# Flush initial readings
time.sleep(5)
ser.reset_input_buffer()


############################# HELPER FUNCTIONS #######################################
def dist2Ticks(dist):
	return int((20/(np.pi*0.065)) * dist)


def deg2Ticks(deg):
	return int((20/(np.pi*0.065)) * (0.075*np.deg2rad(deg)))

def ticks2dist(ticks):
	return int(((np.pi*0.065)/20) * ticks)

def getIMUAngle(ser):
	# Check if there is data in the input buffer
	if (ser.in_waiting > 0):
		# Read serial stream
		angle = ser.readline()     
		
		# Read serial stream
		line = ser.readline()

		# Strip newline and return carriage from line
		line = line.rstrip().lstrip()

		# Convert line to string, strip non-numeric characters and convert to float
		line = str(line)
		line = line.strip("'").strip("b'")
		angle = float(line)

		return angle
	
	else:
		return None

def plot(X,Y):
	plt.plot(X,Y)

##################### DRIVE FUNCTIONS ###########################

## Stop
def stopDriving():
	# Set all motor driver pins low
	gpio.output(31, False)
	gpio.output(33, False)
	gpio.output(35, False)
	gpio.output(37, False)


### Directions
def driveForward():
	global dutyCycle, counterBR, counterFL, leftPWMPin, rightPWMPin
	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)

	# Right Wheels
	gpio.output(35, False)
	gpio.output(37, True)	

	if (counterBR > counterFL):
		leftPWMPin.ChangeDutyCycle(dutyCycle + diff)
		rightPWMPin.ChangeDutyCycle(dutyCycle - diff)
	elif (counterBR < counterFL):
		leftPWMPin.ChangeDutyCycle(dutyCycle - diff)
		rightPWMPin.ChangeDutyCycle(dutyCycle + diff)
	else:
		leftPWMPin.ChangeDutyCycle(dutyCycle)
		rightPWMPin.ChangeDutyCycle(dutyCycle)



def driveBackward():
	global dutyCycle, counterBR, counterFL, leftPWMPin, rightPWMPin
	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)

	# Right Wheels
	gpio.output(35, True)
	gpio.output(37, False)	

	if (counterBR > counterFL):
		leftPWMPin.ChangeDutyCycle(dutyCycle + diff)
		rightPWMPin.ChangeDutyCycle(dutyCycle - diff)
	elif (counterBR < counterFL):
		leftPWMPin.ChangeDutyCycle(dutyCycle - diff)
		rightPWMPin.ChangeDutyCycle(dutyCycle + diff)
	else:
		leftPWMPin.ChangeDutyCycle(dutyCycle)
		rightPWMPin.ChangeDutyCycle(dutyCycle)


def turnRight():
	global dutyCycle, counterBR, counterFL, leftPWMPin, rightPWMPin
	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)

	# Right Wheels
	gpio.output(35, True)
	gpio.output(37, False)	

	if (counterBR > counterFL):
		leftPWMPin.ChangeDutyCycle(dutyCycle + diff)
		rightPWMPin.ChangeDutyCycle(dutyCycle - diff)
	elif (counterBR < counterFL):
		leftPWMPin.ChangeDutyCycle(dutyCycle - diff)
		rightPWMPin.ChangeDutyCycle(dutyCycle + diff)
	else:
		leftPWMPin.ChangeDutyCycle(dutyCycle)
		rightPWMPin.ChangeDutyCycle(dutyCycle)


def turnLeft():
	global dutyCycle, counterBR, counterFL, leftPWMPin, rightPWMPin
	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)

	# Right Wheels
	gpio.output(35, False)
	gpio.output(37, True)	

	if (counterBR > counterFL):
		leftPWMPin.ChangeDutyCycle(dutyCycle + diff)
		rightPWMPin.ChangeDutyCycle(dutyCycle - diff)
	elif (counterBR < counterFL):
		leftPWMPin.ChangeDutyCycle(dutyCycle - diff)
		rightPWMPin.ChangeDutyCycle(dutyCycle + diff)
	else:
		leftPWMPin.ChangeDutyCycle(dutyCycle)
		rightPWMPin.ChangeDutyCycle(dutyCycle)


if __name__ == '__main__':
	##### Initialize GPIO pins ####
	gpio.setmode(gpio.BOARD)
	gpio.setup(31, gpio.OUT)  	# IN1
	gpio.setup(33, gpio.OUT) 	# IN2
	gpio.setup(35, gpio.OUT) 	# IN3
	gpio.setup(37, gpio.OUT) 	# IN4
	gpio.setup(38, gpio.OUT) 	# Left motor PWM pin
	gpio.setup(40, gpio.OUT) 	# Right motor PWM pin

	
	leftPWMPin = gpio.PWM(38,50)
	rightPWMPin = gpio.PWM(40,50) 

	leftPWMPin.start(0)
	rightPWMPin.start(0)

	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP) # back right encoder
	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP) # front left encoder

	dutyCycle = 15
	diff = 5

	counterBR = np.uint64(0)
	counterFL = np.uint64(0)

	currentHeading = 0

	buttonBR = int(0)
	buttonFL = int(0)

	stopDriving()

	while(videoCapture.isOpened()):
		ret, image = videoCapture.read()
		if ret == True:
			# Flip the frame both horizontally and vertically
			image = cv2.flip(image, -1)

			'''
			# Start with open sequence
			for i in range(len(fullSequence)):
				clawDutyCycle = openSequence[i]
				claw.ChangeDutyCycle(clawDutyCycle)
				time.sleep(2)
			'''

			# Convert to HSV and perform thresholding for green objects
			imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
			blackMask = cv2.inRange(imageHSV, minHSV, maxHSV)

			cv2.imshow('Threshold', blackMask)
			cv2.waitKey(0)

			#gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			#gray = cv2.GaussianBlur(blackMask, (7, 7), 0)
			thresh = cv2.erode(blackMask, None, iterations=3)
			thresh = cv2.dilate(thresh, None, iterations=1)
			ret, thresh = cv2.threshold(thresh,230,255,cv2.THRESH_BINARY)
			
			# perform edge detection, then perform a dilation + erosion to
			# close gaps in between object edges
			#edged = cv2.Canny(thresh, 50, 100)
			

			cv2.imshow('Threshold', thresh)
			cv2.waitKey(0)
			# Exit if the user presses 'q'
			#if cv2.waitKey(1) & 0xFF == ord('q'):
				#break


			# Draw contours around detected object
			cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
			cnts = imutils.grab_contours(cnts)
			cX_object = 0
			cY_object = 0
			cX_frame = int(640/2) # verify if this needs switching
			cY_frame = int(480/2) # verify if this needs switching

			for c in cnts:
				# compute the center of the contour
				M = cv2.moments(c)
				cX_object = int(M["m10"] / M["m00"])
				cY_object = int(M["m01"] / M["m00"])
				# draw the contour and center of the shape on the image
				cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
				cv2.circle(image, (cX_object, cY_object), 7, (255, 255, 255), -1)

			cv2.imshow('Threshold', image)
			cv2.waitKey(0)
			
			angle = (cX_frame - cX_object)*0.061
			print(angle)
			if angle < 0:
				# Turn left if angle is negative
				currentHeading = getIMUAngle(ser)
				if (currentHeading == None):
					continue
				desiredHeading = currentHeading + angle

				while (currentHeading > desiredHeading):
					turnLeft()
					currentHeading = getIMUAngle(ser)
				
				stopDriving()
				currentAngle = currentAngle + desiredHeading 
			else: 
				currentHeading = getIMUAngle(ser)
				if (currentHeading == None):
					continue
				desiredHeading = currentHeading + angle

				while (currentHeading < desiredHeading):
					turnRight()
					currentHeading = getIMUAngle(ser)
				
				stopDriving()
				currentAngle = currentAngle + desiredHeading
			
			'''
			# Move in the direction of the object
			distance = dist.euclidean((cX_frame, cY_frame), (cX_object, cY_object))
			ticks = dist2Ticks(float(distance))
			counterBR = 0
			counterFL = 0

			while (counterBR < ticks or counterFL < ticks):
				if (gpio.input(12) != buttonBR):
					buttonBR = int(gpio.input(12)) #holds the state
					counterBR += 1

				if (gpio.input(7) != buttonFL):
					buttonFL = int(gpio.input(7)) #holds the state
					counterFL += 1

				driveForward()

			stopDriving()

			realDistance = ticks2dist((counterBR + counterFL)/2)
			y_val = realDistance*(math.sin(math.radians(currentAngle)))
			x_val = realDistance*(math.cos(math.radians(currentAngle))) 
			Y.append(y_val)
			X.append(x_val)

			# End with close sequence
			for i in range(len(fullSequence)):
				clawDutyCycle = closeSequence[i]
				claw.ChangeDutyCycle(clawDutyCycle)
				time.sleep(2)

			f.write(str(x_val)+" "+str(y_val))

			# Exit if the user presses 'q'
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			'''

	leftPWMPin.stop()
	rightPWMPin.stop()
	stopDriving()
	gpio.cleanup()

	# Release video and file object handles
	videoCapture.release()
	#out.release()
	#f.close()
	################################# PLOT TRAJECTORY #######################
	#plt.plot(X,Y)
	#plt.show()
	#########################################################################
	print('Video closed')
