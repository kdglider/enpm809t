'''
Copyright (c) 2020 Hao Da (Kevin) Dong, Krithika Govindaraj
@file       imuTeleoperation.py
@date       2020/05/01
@brief      Teleoperation of the Baron, using both encoders and IMU for localization
@license    This project is released under the BSD-3-Clause license.
'''

import RPi.GPIO as gpio
import time
import numpy as np
import matplotlib.pyplot as plt
import serial


################################## SERIAL CONNECTION ###########################
ser = serial.Serial('/dev/ttyUSB0', 9600)
count = 0


def dist2Ticks(dist):
	return int((20/(np.pi*0.065)) * dist)

def deg2Ticks(deg):
	return int((20/(np.pi*0.065)) * (0.075*np.deg2rad(deg)))


################################## WHEEL MOTOR CONTROL ###########################

## Stop
def stopDriving():
	pwm1.stop()
	pwm2.stop()

	# Set all motor driver pins low
	gpio.output(31, False)
	gpio.output(33, False)
	gpio.output(35, False)
	gpio.output(37, False)


### Directions
def driveForward():
	global dutyCycle, counterBR, counterFL, pwm1, pwm2
	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)

	# Right Wheels
	gpio.output(35, False)
	gpio.output(37, True)	

	if (counterBR > counterFL):
		pwm1.ChangeDutyCycle(dutyCycle - diff)
		pwm2.ChangeDutyCycle(dutyCycle + diff)
	elif (counterBR < counterFL):
		pwm1.ChangeDutyCycle(dutyCycle - diff)
		pwm2.ChangeDutyCycle(dutyCycle + diff)
	else:
		pwm1.ChangeDutyCycle(dutyCycle)
		pwm2.ChangeDutyCycle(dutyCycle)



def driveBackward():
	global dutyCycle, counterBR, counterFL, pwm1, pwm2
	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)

	# Right Wheels
	gpio.output(35, True)
	gpio.output(37, False)	

	if (counterBR > counterFL):
		pwm1.ChangeDutyCycle(dutyCycle - diff)
		pwm2.ChangeDutyCycle(dutyCycle + diff)
	elif (counterBR < counterFL):
		pwm1.ChangeDutyCycle(dutyCycle - diff)
		pwm2.ChangeDutyCycle(dutyCycle + diff)
	else:
		pwm1.ChangeDutyCycle(dutyCycle)
		pwm2.ChangeDutyCycle(dutyCycle)


def turnRight():
	global dutyCycle, counterBR, counterFL, pwm1, pwm2
	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)

	# Right Wheels
	gpio.output(35, True)
	gpio.output(37, False)	

	if (counterBR > counterFL):
		pwm1.ChangeDutyCycle(dutyCycle - diff)
		pwm2.ChangeDutyCycle(dutyCycle + diff)
	elif (counterBR < counterFL):
		pwm1.ChangeDutyCycle(dutyCycle - diff)
		pwm2.ChangeDutyCycle(dutyCycle + diff)
	else:
		pwm1.ChangeDutyCycle(dutyCycle)
		pwm2.ChangeDutyCycle(dutyCycle)


def turnLeft():
	global dutyCycle, counterBR, counterFL, pwm1, pwm2
	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)

	# Right Wheels
	gpio.output(35, False)
	gpio.output(37, True)	

	if (counterBR > counterFL):
		pwm1.ChangeDutyCycle(dutyCycle - diff)
		pwm2.ChangeDutyCycle(dutyCycle + diff)
	elif (counterBR < counterFL):
		pwm1.ChangeDutyCycle(dutyCycle - diff)
		pwm2.ChangeDutyCycle(dutyCycle + diff)
	else:
		pwm1.ChangeDutyCycle(dutyCycle)
		pwm2.ChangeDutyCycle(dutyCycle)


if __name__ == '__main__':
	##### Initialize GPIO pins ####
	gpio.setmode(gpio.BOARD)
	gpio.setup(31, gpio.OUT)  #IN1
	gpio.setup(33, gpio.OUT) #IN2
	gpio.setup(35, gpio.OUT) #IN3
	gpio.setup(37, gpio.OUT) #IN4

	pwm1 = gpio.PWM(31,50) # BackRight
	pwm2 = gpio.PWM(37,50)

	pwm1.start(0)
	pwm2.start(0)

	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP) # back right encoder
	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP) # front left encoder

	dutyCycle = 15
	diff = 5

	counterBR = np.uint64(0)
	counterFL = np.uint64(0)

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

		elif (driveMode == 'a'):
			angle = input('Enter angle in degrees: ')
			while(True):
				if(ser.in_waiting > 0):
					count += 1

					#Read serial stream
					line = ser.readline()
				

					# Avoid first n lines of serial information
					if count > 10:

						# Strip serial stream of extra characters

						line = line.rstrip().lstrip()
						line = str(line)
						line = line.strip("'")
						line = line.strip("b'")

						#Return float
						line = float(line)
						turnRight()

						if(line > angle): 
							break;	
			stopDriving()

		elif (driveMode == 'd'):
			angle = input('Enter angle in degrees: ')

			while(True):
				if(ser.in_waiting > 0):
					count += 1

					#Read serial stream
					line = ser.readline()
				

					# Avoid first n lines of serial information
					if count > 10:

						# Strip serial stream of extra characters

						line = line.rstrip().lstrip()
						line = str(line)
						line = line.strip("'")
						line = line.strip("b'")

						#Return float
						line = float(line)
						turnRight()

						if(line > angle): 
							break;	
			stopDriving()

		elif (driveMode == 'q'):
			break

		else: 
			print("Invalid Key Pressed!")

	stopDriving()
	gpio.cleanup()
