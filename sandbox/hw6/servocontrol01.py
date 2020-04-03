import cv2
import numpy as np
import Rpi.GPIO as GPIO
import time
import matplotlib.pyplt as plt

	# Create a VideoCapture object and read from input file
	# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture(0)
out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))

GPIO.setmode(GPIO.BOARD)
GPIO.setup(36,GPIO.OUT)
pwm = GPIO.PWM(36,50)

def get_frame(duty_cycle):

	ret, frame = cap.read()
	if ret == True:

		
		pwm.start(duty_cycle)
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



################### MAIN #############################		

	# Check if camera opened successfully
if (cap.isOpened()== False): 
print("Error opening video stream or file")

# Read until video is completed
while(cap.isOpened()):
# Capture frame-by-frame
	get_frame(5) % left 

	get_frame(7.5) % center


# When everything done, release the video capture object
cap.release()
pwm.stop()
GPIO.cleanup()

# Closes all the frames
cv2.destroyAllWindows()


