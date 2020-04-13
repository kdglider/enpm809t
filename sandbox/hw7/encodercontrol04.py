import Rpi.GPIO as gpio
import time
import numpy as np

import matplotlib.pyplot as plt



statesRight = []
statesLeft = []

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


########### Main Code #########
init()

counterBR = np.uint64(0)
counterFL = np.uint64(0)


buttonBR = int(0)
buttonFL = int(0)

# Initialize pwm signal to control motor
pwm1 = gpio.PWM(31,50) # BackRight
pwm2 = gpio.PWM(37,50)
val = 22
pwm1.start(val)
pwm2.start(val)
time.sleep(0.1)

for i in range(0, 100000):
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
		

	if counterBR >= 20 and counterFL >= 20:
		pwm1.stop()
		pwm2.stop()
		gameover()
		saveToFile("enc03Right.txt", statesRight)
		saveToFile("enc03Left.txt" statesLeft)
		plotVal(statesRight)
		plotVal(statesLeft)
		print("Thanks for playing !")
		break


	if counterBR >= 20:
		pwm2.stop()
		gameover()
		saveToFile("enc03Right.txt", statesRight)
		plotVal(statesRight)
		print("Thanks for playing !")
		break

	if counterFL >= 20:
		pwm1.stop()
		gameover()
		saveToFile("enc03Left.txt" statesLeft)
		plotVal(statesLeft)
		print("Thanks for playing !")
		break