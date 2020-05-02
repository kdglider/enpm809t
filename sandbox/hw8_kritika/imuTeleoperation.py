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
import matplotlib.pyplot as plt
import math

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
ser.reset_input_buffer 


############################# HELPER FUNCTIONS #######################################
def dist2Ticks(dist):
	return int((20/(np.pi*0.065)) * dist)


def deg2Ticks(deg):
	return int((20/(np.pi*0.065)) * (0.075*np.deg2rad(deg)))


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

	while(True): 
		driveMode = input('Enter Drive Direction: ')

		if (driveMode == 'w'):
			distance = input('Enter distance in meters: ')
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
			Y.append(distance*(math.sin(math.radians(currentAngle))))
			X.append(distance*(math.cos(math.radians(currentAngle))))

		elif (driveMode == 's'):
			distance = input('Enter distance in meters: ')
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

				driveBackward()
			
			stopDriving()
			Y.append(distance*(math.sin(math.radians(currentAngle))))
			X.append(distance*(math.cos(math.radians(currentAngle))))

		elif (driveMode == 'a'):
			angle = input('Enter angle in degrees: ')

			currentHeading = getIMUAngle(ser)
			desiredHeading = currentHeading - angle

			while (currentHeading > desiredHeading):
				turnLeft()
				currentHeading = getIMUAngle(ser)
			
			stopDriving()
			currentAngle = currentAngle + desiredHeading


		elif (driveMode == 'd'):
			angle = input('Enter angle in degrees: ')

			currentHeading = getIMUAngle(ser)
			desiredHeading = currentHeading + angle

			while (currentHeading < desiredHeading):
				turnRight()
				currentHeading = getIMUAngle(ser)
			
			stopDriving()
			currentAngle = currentAngle + desiredHeading

		elif (driveMode == 'q'):
			break

		else: 
			print("Invalid Key Pressed!")


	leftPWMPin.stop()
	rightPWMPin.stop()
	stopDriving()
	gpio.cleanup()

