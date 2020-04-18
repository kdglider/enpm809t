'''
Copyright (c) 2020 Hao Da (Kevin) Dong, Krithika Govindaraj
@file       servocontrol01.py
@date       2020/04/01
@brief      Records a video of the Baron's claw movements with PWM duty cycle
@license    This project is released under the BSD-3-Clause license.
'''

import RPi.GPIO as GPIO
import numpy as np
import cv2
import time


# Choose whether or not to save the output video
saveVideo = True
resolution = (1280, 720)

 # Define video codec and output file if video needs to be saved
if (saveVideo == True):
	fourcc = cv2.VideoWriter_fourcc(*'mp4v')
	# 720p 30fps video
	out = cv2.VideoWriter('output.mp4', fourcc, 1, resolution)

# Claw parameters
clawPin = 36
closePWM = 5.5
openPWM = 9

# Setup RPi GPIO pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(clawPin, GPIO.OUT)
claw = GPIO.PWM(clawPin, 50)		# Set PWM to 50 Hz

# Start claw in closed position
claw.start(closePWM)

# Create video capture object
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Create claw PWM duty cycle sequence
openSequence = np.arange(closePWM, openPWM+0.5, 0.5)
closeSequence = np.flip(openSequence)
fullSequence = np.hstack((openSequence, closeSequence))

# Open and close claw until user specifies an exit
while (cap.isOpened()): 
	exitCondition = 0

	# Run claw through the open/close sequence
	for i in range(len(fullSequence)):
		dutyCycle = fullSequence[i]
		claw.ChangeDutyCycle(dutyCycle)
		time.sleep(2)

		ret, frame = cap.read()
		if ret == True:			
			# Flip the frame both horizontally and vertically
			frame = cv2.flip(frame, -1)

			# Display the frame and duty cycle
			cv2.putText(frame, 'Duty Cycle: ' + str(dutyCycle) + '%', (10,30), \
						cv2.FONT_HERSHEY_SIMPLEX, 1, \
						color=(0,0,255), thickness=3)
			cv2.imshow('Frame', frame)


			# Save frame to video if desired
			if (saveVideo == True):
				out.write(cv2.resize(frame, resolution))


			# Exit loop if user presses 'q'
			if cv2.waitKey(1) & 0xFF == ord('q'):
				exitCondition = 1
				break

		# If the current frame is invalid, continue to the next one
		else: 
			continue


	# Exit while loop if the user exited from inner for loops
	if (exitCondition == 1):
		break


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


