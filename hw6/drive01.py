import cv2
import numpy as np
import Rpi.GPIO as GPIO
import time
import matplotlib.pyplt as plt


######################## SETUP #################################
##### Ultrasonic 
# Define pin allocations for ultrasonic
trig = 16
echo = 18
# Setup GPIO board & pins
gpio.setmode(gpio.BOARD)
gpio.setup(trig, gpio.OUT)
gpio.setup(echo, gpio.IN)



######################## ULTRASONIC ############################

def distance():

	# Ensure output has no value 
	gpio.output(trig, False)
	time.sleep(0.01)

	# Generate trigger pulse
	gpio.output(trig, True)
	time.sleep(0.00001)
	gpio.output(trig, False)

	# Generate echo time signal
	while gpio.input(echo) == 0:
		pulse_start =  time.time()

	while gpio.input(echo) == 1:
		pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start

	# Convert time to distance
	distance = pulse_duration*17150
	distance = round(distance, 2)

	# Cleanup gpio & return distance estimate
	gpio.cleanup()
	return distance


################################## WHEEL MOTOR CONTROL ###########################
## Initialize 
def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(31,gpio.OUT) #IN1
	gpio.setup(33,gpio.OUT)	#IN2
	gpio.setup(35,gpio.OUT)	#IN3
	gpio.setup(37,gpio.OUT) #IN4
## Stop
def gameover():
	# set all pins low
	gpio.output(31,False)
	gpio.output(33,False)
	gpio.output(35,False)
	gpio.output(37,False)

### Directions
def forward(tf):
	init()
	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)
	# Right Wheels
	gpio.output(31, False)
	gpio.output(33, True)
	# Wait
	time.sleep(tf)
	# Send all pins to low and cleanup
	gameover()
	gpio.cleanup()

def backward(tf):
	init()
	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)
	# Right Wheels
	gpio.output(31, True)
	gpio.output(33, False)
	# Wait
	time.sleep(tf)
	# Send all pins to low and cleanup
	gameover()
	gpio.cleanup()

def left(tf):
	init()
	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)
	# Right Wheels
	gpio.output(31, True)
	gpio.output(33, False)
	# Wait
	time.sleep(tf)
	# Send all pins to low and cleanup
	gameover()
	gpio.cleanup()

def right(tf):
	init()
	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)
	# Right Wheels
	gpio.output(31, False)
	gpio.output(33, True)
	# Wait
	time.sleep(tf)
	# Send all pins to low and cleanup
	gameover()
	gpio.cleanup()


def key_input(event):
	init()

	print("Key: ", event)
	key_press = event
	tf = 1
	if key_press.lower() == 'w':
		forward(tf)
	elif key_press.lower() == 'r':
		reverse(tf)
	elif key_press. lower() == 'a':
		pivotleft(tf)
	elif key_press.lower() == 's':
		pivotright(tf)
	else: 
		print("Invalid Key Pressed !")


while True:
	time.sleep(1)
	print("Distance ", distance(), "cm")
	key_press = input("Select driving mode: ")
	if key_press == 'q':
		break
	key_input(key_press)
