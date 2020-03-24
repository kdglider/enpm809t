import RPi.GPIO as gpio
import time
import picamera
import cv2
import numpy as np
import os

# Define pin allocations
trig = 16
echo = 18

def captureAndSaveImage(distance):
    name = "image.jpg"
    os.system('raspistill -w 640 -h 480 -hf -vf -o ' + name)
    """
    with picamera.PiCamera() as camera:
        camera.resolution =(1024,768)
        camera.start_preview()
        time.sleep(2)
        camera.capture('image.jpg')
    """    
    img = cv2.imread('image.jpg')
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    red = (0,0,255)
    cv2.putText(img,str(distance)+" cm",(100,100),font, 1, red,1)
    cv2.imwrite('image.jpg', img)

def distance():
	
	gpio.setmode(gpio.BOARD) #physical numbering
	gpio.setup(trig, gpio.OUT) # Sendig 3.3V 
	gpio.setup(echo, gpio.IN)
	# Ensure trig pin has no output by setting low
	gpio.output(trig,False)
	time.sleep(0.01) # time in seconds
	# Generate trig pulse
	gpio.output(trig, True)
	time.sleep(0.000010)
	gpio.output(trig,False)
	# Generate echo signal
	while gpio.input(echo) == 0:
		pulse_start = time.time()
	while gpio.input(echo) == 1:
		pulse_end =  time.time()
	pulse_duration = pulse_end - pulse_start

	# Convert time to distance
	distance =  pulse_duration * 17150 
	distance = round(distance,2)
	
	# Cleanup and return distance estimate
	gpio.cleanup()
	return distance
    
def main():
    distances = []
    for i in range(1,10):
        distances.append(distance())
    meanDistance = np.mean(distances)
    captureAndSaveImage(meanDistance)
        
main()  


