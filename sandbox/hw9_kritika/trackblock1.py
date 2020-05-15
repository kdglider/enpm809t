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
import math
import cv2
import imutils

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


############################### IMU SERIAL CONNECTION ############################
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
	print(line)
	angle = float(line)

	return angle

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


def turnRight(turnAngle):
	global dutyCycle, leftPWMPin, rightPWMPin, ser
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

	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP) # back right encoder
	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP) # front left encoder

	dutyCycle = 75
	diff = 10

	counterBR = np.uint64(0)
	counterFL = np.uint64(0)

	currentHeading = 0

	buttonBR = int(0)
	buttonFL = int(0)

	stopDriving()

	while(videoCapture.isOpened()):
		ret, image = videoCapture.read()
		ret, image = videoCapture.read()
		ret, image = videoCapture.read()

		if ret == True:
			# Flip the frame both horizontally and vertically
			image = cv2.flip(image, -1)

			# Convert to HSV and perform thresholding for green objects
			imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
			blackMask = cv2.inRange(imageHSV, minHSV, maxHSV)

			#cv2.imshow('Threshold', blackMask)
			#cv2.waitKey(0)

			#gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			#gray = cv2.GaussianBlur(blackMask, (7, 7), 0)
			thresh = cv2.erode(blackMask, None, iterations=3)
			thresh = cv2.dilate(thresh, None, iterations=1)
			ret, thresh = cv2.threshold(thresh,230,255,cv2.THRESH_BINARY)

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
			# Exit if the user presses 'q'
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			#key = cv2.waitKey(0)

			angle = (cX_object - cX_frame)*0.061
			if (abs(angle) < 2):
				continue
	
			print(angle)
			if angle < 0:
				# Turn left if angle is negative
				turnLeft(-angle)
			else: 
				turnRight(angle)

	leftPWMPin.stop()
	rightPWMPin.stop()
	stopDriving()
	gpio.cleanup()

	# Release video and file object handles
	videoCapture.release()

	print('Video object handle closed')
