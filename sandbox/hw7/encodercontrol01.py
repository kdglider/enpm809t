import Rpi.GPIO as gpio
import numpy as numpy

filename = "enc01.txt"
states = []

##### Initialize GPIO pins ####
def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(12,gpio.IN, pull_up_down = gpio.PUD_UP)

def gameover():
	gpio.cleanup()

def saveToFile(filename, states):
	f = open(filename, "w+")
	for i in range(len(states)):
		f.write(str(states[i])+"\n")
	f.close()


########### Main Code #########
init()

counter = np.uint64(0)
button = int(0)

while True:

	if int(gpio.input(12) != int(button)):
		button = int(gpio.input(12))
		states.append(button)
		counter += 1
		print(counter)

	if counter >= 20:
		gameover()
		saveToFile(filename, states)
		print("Thanks for playing !")
		break