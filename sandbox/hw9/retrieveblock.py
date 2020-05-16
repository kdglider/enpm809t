'''
Copyright (c) 2020 Hao Da (Kevin) Dong, Krithika Govindaraj
@file       retrieveblock.py
@date       2020/05/15
@brief      Autonomous object recognition and retrieval with Baron robot
@license    This project is released under the BSD-3-Clause license.
'''

import RPi.GPIO as gpio
import serial
import time
import numpy as np
import math
import cv2
import imutils

############################## EMAIL SETUP ################################
import os
from datetime import datetime
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

'''
#Define time stamp and record an image
pic_time = datetime.now().strftime('%Y%m%d%H%M%S')
command = 'raspistill -w 1280 -h 720 -vf -hf -o '+ pic_time +'.jpg' 
os.system(command)

# Email information
smtpUser = 'your email'
smtpPass = 'your pass'

# Destination email information
toAdd = ['krithideepu@gmail.com', 'your email'] #Add these email after code works 
# ENPM809TS19@gmail.com, skotasai@umd.edu
fromAdd = smtpUser 
subject = 'Image recorded at '+ pic_time
msg = MIMEMultipart()
msg['Subject'] = subject
msg['From'] = fromAdd
msg['To'] = toAdd
msg.preamble = "Image recorded at "+ pic_time
'''

'''
############################## FILE TO STORE COORDINATES ##################
f = open("demofile.txt", "a")

################################## TRAJECTORY ########################################
X = [0] # X coords
Y = [0]	# Y coords
# forwardCoord = "Y"  initially if we move forward we are moving in Y
# backCoord = "Y"
# rightCoord = "X"   # initially if we move right we are moving in X
# leftCoord = "X"	   # initially if we move left we are moving in X
currentAngle = 90      # facing north
'''

############################# HELPER FUNCTIONS #######################################
def dist2Ticks(dist):
	return int((20/(np.pi*0.065)) * dist)

def deg2Ticks(deg):
	return int((20/(np.pi*0.065)) * (0.075*np.deg2rad(deg)))

def ticks2dist(ticks):
	return int(((np.pi*0.065)/20) * ticks)

def getIMUAngle():
	global ser
	ser.reset_input_buffer()
	while (ser.in_waiting == 0):
		continue

	# Read serial stream
	line = ser.readline()
	#print(line)

	# Strip newline and return carriage from line
	line = line.rstrip().lstrip()

	# Convert line to string, strip non-numeric characters and convert to float
	line = str(line)
	line = line.strip("'").strip("b'")
	#print(line)
	angle = float(line)

	return angle


def getObjectLocation(image):
	sensorSizeY = 2.76	# mm
	sensorResY = 900	# px
	focalLength = 3.04	# mm
	pixPerMil = sensorResY / sensorSizeY	# px/mm
	objectHeight = 3.5	# cm

	# Set HSV limits for thresholding
	minHSV = np.array([100, 100, 0])
	maxHSV = np.array([125, 255, 255])
	#minHSV = np.array([0, 0, 0])
	#maxHSV = np.array([255, 170, 50])

	imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	blackMask = cv2.inRange(imageHSV, minHSV, maxHSV)

	thresh = cv2.erode(blackMask, None, iterations=3)
	thresh2 = cv2.dilate(thresh, None, iterations=1)
	ret, thresh3 = cv2.threshold(thresh2, 230, 255, cv2.THRESH_BINARY)
	
	
	#cv2.imshow('a', image)
	#cv2.imshow('1', thresh)
	#cv2.imshow('2', thresh2)
	#cv2.imshow('3', thresh3)
	#cv2.waitKey(0)
	

	x, y, w, h = cv2.boundingRect(thresh3)

	# Draw contours around detected object
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	cX_object = 0
	cY_object = 0
	cX_frame = int(640/2) # verify if this needs switching
	cY_frame = int(480/2) # verify if this needs switching

	if (h == 0):
		print('No object detected')
		return None, None, None

	for c in cnts:
		# compute the center of the contour
		M = cv2.moments(c)
		cX_object = int(M["m10"] / M["m00"])
		cY_object = int(M["m01"] / M["m00"])
		# draw the contour and center of the shape on the image
		cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
		cv2.circle(image, (cX_object, cY_object), 7, (255, 255, 255), -1)

	angle = (cX_object - cX_frame) * 0.061
	distance = objectHeight * focalLength * pixPerMil / h
	
	cv2.putText(image, 'Distance: ' + str(distance) + 'cm', (10,30), \
				cv2.FONT_HERSHEY_SIMPLEX, 1, \
				color=(0,0,255), thickness=3)

	return angle, distance, image

##################### DRIVE FUNCTIONS ###########################
## Stop
def stopDriving():
	# Set all motor driver pins low
	gpio.output(31, False)
	gpio.output(33, False)
	gpio.output(35, False)
	gpio.output(37, False)


### Directions
def driveForward(distance):
	global dutyCycle, diff, leftPWMPin, rightPWMPin

	ticks = dist2Ticks(distance)
	counterBR = 0
	counterFL = 0
	buttonBR = int(0)
	buttonFL = int(0)

	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)

	# Right Wheels
	gpio.output(35, False)
	gpio.output(37, True)

	while (counterBR < ticks or counterFL < ticks):
		# Move in the direction of the object
		if (counterBR > counterFL):
			leftPWMPin.ChangeDutyCycle(dutyCycle + diff)
			rightPWMPin.ChangeDutyCycle(dutyCycle - diff)
		elif (counterBR < counterFL):
			leftPWMPin.ChangeDutyCycle(dutyCycle - diff)
			rightPWMPin.ChangeDutyCycle(dutyCycle + diff)
		else:
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
		
		# Update encoder states
		if (gpio.input(12) != buttonBR):
			buttonBR = int(gpio.input(12)) #holds the state
			counterBR += 1

		if (gpio.input(7) != buttonFL):
			buttonFL = int(gpio.input(7)) #holds the state
			counterFL += 1
	
	stopDriving()


def driveBackward(distance):
	global dutyCycle, diff, leftPWMPin, rightPWMPin

	ticks = dist2Ticks(distance)
	counterBR = 0
	counterFL = 0
	buttonBR = int(0)
	buttonFL = int(0)

	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)

	# Right Wheels
	gpio.output(35, True)
	gpio.output(37, False)	

	while (counterBR < ticks or counterFL < ticks):
		# Move in the direction of the object
		if (counterBR > counterFL):
			leftPWMPin.ChangeDutyCycle(dutyCycle + diff)
			rightPWMPin.ChangeDutyCycle(dutyCycle - diff)
		elif (counterBR < counterFL):
			leftPWMPin.ChangeDutyCycle(dutyCycle - diff)
			rightPWMPin.ChangeDutyCycle(dutyCycle + diff)
		else:
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
		
		# Update encoder states
		if (gpio.input(12) != buttonBR):
			buttonBR = int(gpio.input(12)) #holds the state
			counterBR += 1

		if (gpio.input(7) != buttonFL):
			buttonFL = int(gpio.input(7)) #holds the state
			counterFL += 1
	
	stopDriving()


def turnRight(turnAngle):
	global dutyCycle, leftPWMPin, rightPWMPin
	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)

	# Right Wheels
	gpio.output(35, True)
	gpio.output(37, False)	
	
	currentHeading = getIMUAngle()
	previousHeading = currentHeading

	desiredHeading = currentHeading + turnAngle

	if (desiredHeading < 360):
		while (currentHeading < desiredHeading):
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
			currentHeading = getIMUAngle()
	
	else:
		while (currentHeading < 360):
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
			previousHeading = currentHeading
			currentHeading = getIMUAngle()
			if (currentHeading < previousHeading):
				break
		while (currentHeading < desiredHeading - 360):
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
			currentHeading = getIMUAngle()
	
	stopDriving()


def turnLeft(turnAngle):
	global dutyCycle, counterBR, counterFL, leftPWMPin, rightPWMPin
	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)

	# Right Wheels
	gpio.output(35, False)
	gpio.output(37, True)	

	currentHeading = getIMUAngle()
	previousHeading = currentHeading
	if (currentHeading == None):
		return None

	desiredHeading = currentHeading - turnAngle

	if (desiredHeading > 0):
		while (currentHeading > desiredHeading):
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
			currentHeading = getIMUAngle()
	
	else:
		while (currentHeading > 0):
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
			previousHeading = currentHeading
			currentHeading = getIMUAngle()
			if (currentHeading > previousHeading):
				break
		while (currentHeading > desiredHeading + 360):
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
			currentHeading = getIMUAngle()
	
	stopDriving()


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

	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP) 	# Back right encoder
	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP) 		# Front left encoder

	dutyCycle = 70
	diff = 10

	################################# CLAW OPERATION ##############################
	# Claw parameters
	clawPin = 36
	closePWM = 5.5
	openPWM = 9

	# Setup RPi GPIO pins
	gpio.setup(clawPin, gpio.OUT)
	claw = gpio.PWM(clawPin, 50)		# Set PWM to 50 Hz

	# Start claw in open position
	claw.start(openPWM)

	############################### IMU SERIAL CONNECTION ############################
	# Create serial connection
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	# Flush initial readings
	time.sleep(5)
	ser.reset_input_buffer()

	############################## VIDEO SETTINGS #############################

	# Set desired video resolution, framerate and logging offset
	resolution = (1280,720)
	fps = 15

	# Create video capture object 
	videoCapture = cv2.VideoCapture(0)
	videoCapture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

	# Set HSV limits for thresholding
	minHSV = np.array([100, 152, 0])
	maxHSV = np.array([125, 255, 255])

	stopDriving()

	while (True):

		retrieveCommand = input('Retrieve next object (Enter Y or N)? ')

		if (retrieveCommand == 'Y' or retrieveCommand == 'y'):
			ret, image = videoCapture.read()
			ret, image = videoCapture.read()
			ret, image = videoCapture.read()

			# Flip the frame both horizontally and vertically
			image = cv2.flip(image, -1)
			print(image.shape)

			angle, distanceCM, newImage = getObjectLocation(image)

			if (angle == None):
				continue

			cv2.imshow('Threshold', newImage)
			# Exit if the user presses 'q'
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			
			print(angle)

			# Turn toward object
			if (abs(angle) > 2):
				if angle < 0:
					# Turn left if angle is negative
					turnLeft(-angle)
				else: 
					turnRight(angle)
			
			# Drive to object, pick it up and drive back
			driveForward((distanceCM)/100)

			claw.ChangeDutyCycle(closePWM)
			time.sleep(2)

			driveBackward((distanceCM)/100)

			claw.ChangeDutyCycle(openPWM)

		else:
			break

		'''
		realDistance = ticks2dist((counterBR + counterFL)/2)
		y_val = realDistance*(math.sin(math.radians(currentAngle)))
		x_val = realDistance*(math.cos(math.radians(currentAngle))) 
		Y.append(y_val)
		X.append(x_val)
		f.write(str(x_val)+" "+str(y_val))
		'''
			
	leftPWMPin.stop()
	rightPWMPin.stop()
	claw.stop()
	stopDriving()
	gpio.cleanup()

	# Release video and file object handles
	videoCapture.release()

	print('Video object handle closed')
