import Rpi.GPIO as gpio
import time
import numpy as np

import matplotlib.pyplot as plt



statesRight = []
statesLeft = []

dist = 10 
wheelRev =int( (1/(2*3.14*0.0325)) * dist)
duty_cycle = 22 
counter = 0
button = int(0)
##### Initialize GPIO pins ####
def init():
	counter = 0
	button = int(0)
	gpio.setmode(gpio.BOARD)
	gpio.setup(31,gpio.OUT)  #IN1
 	gpio.setup(33, gpio.OUT) #IN2
 	gpio.setup(35, gpio.OUT) #IN3
 	gpio.setup(37, gpio.OUT) #IN4

 	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP) # back right encoder
 	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP) # front left encoder

def gameover():
	gpio.output(31, FALSE)
	gpio.output(33, FALSE)
	gpio.output(35, FALSE)
	gpio.output(37, FALSE)
	gpio.cleanup()


def getAngleDist(count_forward,count_reverse,rotate_left,rotate_right):
	dist_forward = count_forward*(2*3.14*0.0325)
	dist_reverse = count_reverse*(2*3.14*0.0325)
	angle_left = ((rotate_left/20 )* 360)
	angle_right = ((rotate_right/20 )* 360)
	return dist_forward,dist_reverse,angle_left,angle_right


### Directions
def forward(tf):
	init()
	for i in range(0,49):
		#Left wheels
		pwm1.start(duty_cycle)
		gpio.output(33, False)
		# Right Wheels
		gpio.output(35, False)
		pwm4.start(duty_cycle)
		time.sleep(0.1)

		if int(gpio.input(7) != int(button)):
		button = int(gpio.input(7)) #holds the state
		statesRight.append(button)
		counter += 1

	# Send all pins to low and cleanup
	gameover()
	gpio.cleanup()
	return counter

def backward(tf):
	init()
	for i in range(0, wheelRev):
		#Left wheels
		gpio.output(31, False)
		pwm2.start(duty_cycle)
		# Right Wheels
		pwm3.start(duty_cycle)
		gpio.output(37, False)
		time.sleep(0.1)

		if int(gpio.input(12) != int(buttonBR)):
		button = int(gpio.input(12)) #holds the state
		statesRight.append(button)
		counter += 1

	# Send all pins to low and cleanup
	gameover()
	gpio.cleanup()
	return counter

def left(tf):
	init()
	for i in range(0, wheelRev):
		#Left wheels
		pwm1.start(duty_cycle)
		gpio.output(33, False)
		# Right Wheels
		pwm3.start(duty_cycle)
		gpio.output(37, False)
		time.sleep(0.1)


		if int(gpio.input(7) != int(buttonBR)):
		button = int(gpio.input(7)) #holds the state
		statesRight.append(button)
		counter += 1

		# Send all pins to low and cleanup
	gameover()
	gpio.cleanup()
	return counter

def right(tf):
	init()
	for i in range(0, wheelRev):
		#Left wheels
		gpio.output(31, False)
		pwm2.start(duty_cycle)
		# Right Wheels
		gpio.output(35, False)
		pwm4.start(duty_cycle)
		time.sleep(0.1)

		if int(gpio.input(12) != int(buttonBR)):
		button = int(gpio.input(12)) #holds the state
		statesRight.append(button)
		counter += 1

		# Send all pins to low and cleanup
	gameover()
	gpio.cleanup()
	return counter

def key_input(event):
	init()

	print("Key: ", event)
	key_press = event
	tf = 1
	if key_press.lower() == 'w':
		count_forward = forward(tf)
	elif key_press.lower() == 'r':
		count_reverse= reverse(tf)
	elif key_press. lower() == 'a':
		rotate_left = pivotleft(tf)
	elif key_press.lower() == 's':
		rotate_right = pivotright(tf)
	else: 
		print("Invalid Key Pressed !")
	return count_forward,count_reverse,rotate_left,rotate_right

########### Main Code #########

# Initialize pwm signal to control motor
pwm1 = gpio.PWM(31,50) # BackRight
pwm2 = gpio.PWM(33,50) 
pwm3 = gpio.PWM(35,50) 
pwm4 = gpio.PWM(37,50) # FrontLeft




while(True): 
	key_press = input("Select driving mode: ")


	if key_press == 'q':
		break

	count_forward,count_reverse,rotate_left,rotate_right = key_input(key_press)

	dist_forward,dist_reverse,angle_left,angle_right = getAngleDist(count_forward,count_reverse,rotate_left,rotate_right)
	print("count_forward, count_reverse, rotate_left, rotate_right")
	print(count_forward,count_reverse,rotate_left,rotate_right)
	print("dist_forward, dist_backward, angle_left, angle_right")
	print(dist_forward,dist_reverse,angle_left,angle_right)

