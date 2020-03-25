'''
Copyright (c) 2020 Hao Da (Kevin) Dong, Krithika Govindaraj
@file       hw4p2.py
@date       2020/02/26
@brief      Captures an image and overlays the distance to the object in the image
@license    This project is released under the BSD-3-Clause license.
'''

import RPi.GPIO as gpio
import time
import numpy as np
import cv2
import os

# Define pin allocations
trig = 16
echo = 18

def captureAndSaveImage(distance):
	# Capture image using the terminal command
    name = "image.png"
    os.system('raspistill -w 1280 -h 720 -hf -vf -o ' + name)

	# Read in captured image and overlay distance measurement
    img = cv2.imread('image.png')
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL

    cv2.putText(img, str(distance) + " cm", (10,30), font, 1, color=(0,255,0), thickness=2)

    cv2.imwrite('image.png', img)


def distance():
	# Use the physical board pin numbering scheme
	gpio.setmode(gpio.BOARD)

	# Set trigger and echo pins as output and input
	gpio.setup(trig, gpio.OUT)
	gpio.setup(echo, gpio.IN)

	# Ensure trigger pin has no output by setting it low
	gpio.output(trig, False)
	time.sleep(0.01) 		# Seconds

	# Generate trigger pulse
	gpio.output(trig, True)
	time.sleep(0.000010)
	gpio.output(trig,False)

	# Calculate duration of echo signal
	while (gpio.input(echo) == 0):
		pulseStart = time.time()
	while (gpio.input(echo) == 1):
		pulseEnd =  time.time()
	pulseDuration = pulseEnd - pulseStart

	# Convert time to distance by multiplying by the speed of sound divided by 2
	distance = pulseDuration * (343 * 100)/2
	
	# Clean up GPIO and return distance estimate
	gpio.cleanup()
	return distance
    

if __name__ == '__main__':
	# Average 10 distance readings and overlay on captured image
    distances = []
    for i in range(10):
        distances.append(distance())
    meanDistance = np.mean(distances)

    captureAndSaveImage(round(meanDistance, 1))
    


