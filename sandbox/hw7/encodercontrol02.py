import RPi.GPIO as gpio
import time
import numpy as np

import matplotlib.pyplot as plt


filename = "enc02.txt"
states = []

##### Initialize GPIO pins ####
def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(31, gpio.OUT)  	#IN1
	gpio.setup(33, gpio.OUT) 	#IN2
	gpio.setup(35, gpio.OUT) 	#IN3
	gpio.setup(37, gpio.OUT) 	#IN4
	
	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)

def gameover():
	gpio.output(31, False)
	gpio.output(33, False)
	gpio.output(35, False)
	gpio.output(37, False)
	gpio.cleanup()

def saveToFile(filename, states):
	f = open(filename, "w+")
	for i in range(len(states)):
		f.write(str(states[i])+"\n")
	f.close()

def plotVal(data):
	x = np.arange(len(data))
	plt.plot(x,data)
	plt.show()


########### Main Code #########
init()

counter = np.uint64(0)
button = int(0)

# Initialize pwm signal to control motor
pwm = gpio.PWM(37, 50)
val = 50
pwm.start(val) # pwm input through pin 14
time.sleep(0.1)

while (counter < 20):
	print("counter = ", counter, "GPIO state: ", gpio.input(12)) #Right encoder pin 12
	
	states.append(button)

	if (gpio.input(12) != button):
		button = int(gpio.input(12)) #holds the state
		counter += 1
		print(counter)
		


pwm.stop()
gameover()
saveToFile(filename, states)
plotVal(states)
print("Thanks for playing !")


