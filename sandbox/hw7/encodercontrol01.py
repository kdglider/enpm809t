'''
Copyright (c) 2020 Hao Da (Kevin) Dong, Krithika Govindaraj
@file       encodercontrol01.py
@date       2020/04/01
@brief      Encoder test to record 20 state changes in a text file
@license    This project is released under the BSD-3-Clause license.
'''

import RPi.GPIO as gpio
import numpy as np

filename = "enc01.txt"
states = []

##### Initialize GPIO pins ####
def init():
	# Set GPIO mode to board pin numbering
	gpio.setmode(gpio.BOARD)

	# Set pull-up resistor since the encoder pin is open-drain 
	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)


def gameover():
	gpio.cleanup()


def saveToFile(filename, states):
	f = open(filename, "w+")
	for i in range(len(states)):
		f.write(str(states[i]) + "\n")
	f.close()


if __name__ == '__main__':
	init()

	counter = np.uint64(0)	# Counter to keep track of states
	encoderState = int(0)	# Current encoder state

	while True:
		# Update encoder state if it is different from pin input
		if (int(gpio.input(12)) != int(encoderState)):
			encoderState = int(gpio.input(12))
			states.append(encoderState)			# Record state
			counter += 1						# Update counter
			print(counter)

		# If 20 changes have been reached, save states to file and exit
		if (counter >= 20):
			gameover()
			saveToFile(filename, states)
			print("Thanks for playing!")
			break
