'''
Copyright (c) 2020 Hao Da (Kevin) Dong, Krithika Govindaraj
@file       teleoperation.py
@date       2020/04/01
@brief      Teleoperation of the Baron with an option to record video
@license    This project is released under the BSD-3-Clause license.
'''

import RPi.GPIO as gpio
import numpy as np
import cv2
import time


######################## SETUP #################################

# Choose whether or not to save the output video
saveVideo = False
resolution = (1280, 720)

 # Define video codec and output file if video needs to be saved
if (saveVideo == True):
	fourcc = cv2.VideoWriter_fourcc(*'mp4v')
	# 720p 30fps video
	out = cv2.VideoWriter('output.mp4', fourcc, 30, resolution)

# Define pin allocations for ultrasonic and claw servo
trig = 16
echo = 18
clawPin = 36

# Set GPIO mode to board pin numbering
gpio.setmode(gpio.BOARD)

# Set up ultrasonic pins
gpio.setup(trig, gpio.OUT)
gpio.setup(echo, gpio.IN)

# Set up motor driver pins
gpio.setup(31,gpio.OUT) 	# IN1
gpio.setup(33,gpio.OUT)		# IN2
gpio.setup(35,gpio.OUT)		# IN3
gpio.setup(37,gpio.OUT) 	# IN4

# Set up claw pin and PWM object at 50 Hz.
gpio.setup(clawPin, GPIO.OUT)
claw = GPIO.PWM(clawPin, 50)

# Define claw PWM limits and set to open as the initial position
closePWM = 5.5
openPWM = 9
claw.start(openPWM)

# Create video capture object
cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)


######################## ULTRASONIC ############################

def getDistance():
	# Ensure that trigger is set to low
	gpio.output(trig, False)
	time.sleep(0.01)

	# Generate trigger pulse
	gpio.output(trig, True)
	time.sleep(0.00001)
	gpio.output(trig, False)

	# Read echo signal and calculate duration
	while gpio.input(echo) == 0:
		pulse_start = time.time()

	while gpio.input(echo) == 1:
		pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start

	# Convert time to distance by multiplying by the speed of sound
	distance = pulse_duration * 17150
	distance = round(distance, 2)

	# Return distance estimate
	return distance


####################### CLAW ###################################

def openClaw():
	claw.ChangeDutyCycle(openPWM)


def closeClaw():
	claw.ChangeDutyCycle(closePWM)

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
	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)

	# Right Wheels
	gpio.output(35, False)
	gpio.output(37, True)


def driveBackward():
	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)

	# Right Wheels
	gpio.output(35, True)
	gpio.output(37, False)


def turnLeft():
	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)

	# Right Wheels
	gpio.output(35, True)
	gpio.output(37, False)


def turnRight():
	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)

	# Right Wheels
	gpio.output(35, False)
	gpio.output(37, True)

'''
def key_input(event):
	init()

	print("Key: ", event)
	keyPress = event
	tf = 1
	if keyPress.lower() == 'w':
		forward(tf)
	elif keyPress.lower() == 'r':
		reverse(tf)
	elif keyPress. lower() == 'a':
		pivotleft(tf)
	elif keyPress.lower() == 's':
		pivotright(tf)
	else: 
		print("Invalid Key Pressed !")
'''
#################################### CAMERA ###################
'''
def get_frame(duty_cycle):

	ret, frame = cap.read()
	if ret == True:

		
		claw.start(duty_cycle)
		time.sleep(2)
		

	# Display the resulting frame
		plt.text(0.5, 0.5, str(duty_cycle), fontsize=12)
		plt.imshow('Frame',frame)
		plt.show()
		out.write(frame)


	# Press Q on keyboard to  exit
		if cv2.waitKey(25) & 0xFF == ord('q'):
		  break

	# Break the loop
	else: 
		break
'''

##################################### MAIN ###################

# Check if camera opened successfully
if (cap.isOpened() == False): 
	print("Error opening video stream or file")

# Main loop to run as long as the camera is open
while(cap.isOpened()):
	# Get distance from ultrasonic sensor
	time.sleep(1)
	distance = getDistance()
	print("Distance: " + str(distance) + "cm")

	# Get frome from video capture object
	ret, frame = cap.read()

	# Flip the frame both horizontally and vertically
	frame = cv2.flip(frame, -1)

	# Display the frame and distance
	cv2.putText(frame, 'Distance: ' + str(distance) + 'cm', (10,30), \
				cv2.FONT_HERSHEY_SIMPLEX, 1, \
				color=(0,0,255), thickness=3)
	cv2.imshow('Frame', frame)

	# Save frame to video if desired
	if (saveVideo == True):
		out.write(cv2.resize(frame, resolution))

	# Get direction of locomotion from user
	keyPress = input("Enter control input: ")

	if keyPress.lower() == 'w':
		driveForward()
	elif keyPress.lower() == 's':
		driveBackward()
	elif keyPress. lower() == 'a':
		turnLeft()
	elif keyPress.lower() == 'd':
		turnRight()
	elif (keyPress.lower() == 'o'):
		openClaw()
	elif (keyPress.lower() == 'i'):
		closeClaw()
	elif (keyPress.lower() == 'q'):
		break
	else: 
		print("Invalid Key Pressed!")
	

# Release video and file object handles
cap.release()
if (saveVideo == True):
	out.release()

print('Video and file handles closed')

# Stop PWM signal and clean up GPIO
claw.stop()
GPIO.cleanup()

# Close all windows
cv2.destroyAllWindows()
