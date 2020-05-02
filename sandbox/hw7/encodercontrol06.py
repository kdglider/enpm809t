import RPi.GPIO as gpio
import time
import numpy as np
import matplotlib.pyplot as plt


def dist2Ticks(dist):
	return int((20/(np.pi*0.065)) * dist)

def deg2Ticks(deg):
	return int((20/(np.pi*0.065)) * (0.075*np.deg2rad(deg)))


################################## WHEEL MOTOR CONTROL ###########################

## Stop
def stopDriving():
	

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
		pwm1.ChangeDutyCycle(dutyCycle + diff)
		pwm2.ChangeDutyCycle(dutyCycle - diff)
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
		pwm1.ChangeDutyCycle(dutyCycle + diff)
		pwm2.ChangeDutyCycle(dutyCycle - diff)
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
		pwm1.ChangeDutyCycle(dutyCycle + diff)
		pwm2.ChangeDutyCycle(dutyCycle - diff)
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
		pwm1.ChangeDutyCycle(dutyCycle + diff)
		pwm2.ChangeDutyCycle(dutyCycle - diff)
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
	gpio.setup(38, gpio.OUT) #IN3
	gpio.setup(40, gpio.OUT) #IN4

	pwm1 = gpio.PWM(38,50) # BackRight
	pwm2 = gpio.PWM(40,50)

	pwm1.start(0)
	pwm2.start(0)

	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP) # back right encoder
	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP) # front left encoder

	dutyCycle = 75
	diff = 10

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
			ticks = deg2Ticks(float(angle))

			counterBR = 0
			counterFL = 0

			while (counterBR < ticks or counterFL < ticks):
				if (gpio.input(12) != buttonBR):
					buttonBR = int(gpio.input(12)) #holds the state
					counterBR += 1

				if (gpio.input(7) != buttonFL):
					buttonFL = int(gpio.input(7)) #holds the state
					counterFL += 1

				turnLeft()
			
			stopDriving()

		elif (driveMode == 'd'):
			angle = input('Enter angle in degrees: ')
			ticks = deg2Ticks(float(angle))

			counterBR = 0
			counterFL = 0

			while (counterBR < ticks or counterFL < ticks):
				if (gpio.input(12) != buttonBR):
					buttonBR = int(gpio.input(12)) #holds the state
					counterBR += 1

				if (gpio.input(7) != buttonFL):
					buttonFL = int(gpio.input(7)) #holds the state
					counterFL += 1

				turnRight()
			
			stopDriving()

		elif (driveMode == 'q'):
			break

		else: 
			print("Invalid Key Pressed!")
	
	pwm1.stop()
	pwm2.stop()
	stopDriving()
	gpio.cleanup()

