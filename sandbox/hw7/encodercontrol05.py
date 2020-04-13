import Rpi.GPIO as gpio
import time
import numpy as np

import matplotlib.pyplot as plt



statesRight = []
statesLeft = []
dist = 10 
wheelRev =int( (1/(2*3.14*0.0325)) * dist)

##### Initialize GPIO pins ####
def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(31,gpio.OUT)  #IN1
 	gpio.setup(33, gpio.OUT) #IN2
 	gpio.setup(35, gpio.out) #IN3
 	gpio.setup(37, gpio.out) #IN4

 	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP) # back right encoder
 	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP) # front left encoder

def gameover():
	gpio.output(31, FALSE)
	gpio.output(33, FALSE)
	gpio.output(35, FALSE)
	gpio.output(37, FALSE)
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

def subplot(val1,val2):
fig = plt.figure()
sb1 = fig.add_subplot(1,2,1)
x = np.arange(len(val1))
sb1.plot(x, val1, c='r')
sb1.set_title('Encoder Data')
sb1.set_xlabel('Time')
sb1.set_ylabel('State')

sb2 = fig.add_subplot(1,2,2)
x = np.arange(len(val2))
sb2.plot(x, val1, c='r')
sb2.set_title('Encoder Data')
sb2.set_xlabel('Time')
sb2.set_ylabel('State')

# Display all plots

########### Main Code #########
init()

counterBR = np.uint64(0)
counterFL = np.uint64(0)


buttonBR = int(0)
buttonFL = int(0)

# Initialize pwm signal to control motor
pwm1 = gpio.PWM(31,50) # BackRight
pwm2 = gpio.PWM(37,50) # FrontLeft
val = 22
pwm1.start(val)
pwm2.start(val)
time.sleep(0.1)

for i in range(0, wheelRev):  # from 0 to wheelrevs for 10 meters which is 48.97 
	print("counterBR = ", counterBR,
		  "counterFL = ", counterFL, 
	      "BR state: ", gpio.input(12), 
	      "FL state: ", gpio.input(7)) 

	if int(gpio.input(12) != int(buttonBR)):
		button = int(gpio.input(12)) #holds the state
		statesRight.append(button)
		counter += 1

	if int(gpio.input(7) != int(buttonFL)):
		button = int(gpio.input(7)) #holds the state
		statesLeft.append(button)
		counter += 1
		
	pwm1.stop()
	pwm2.stop()
	gameover()
	saveToFile("enc03Right.txt", statesRight)
	saveToFile("enc03Left.txt" statesLeft)
	subplot(statesRight, statesLeft)
	print("Thanks for playing !")