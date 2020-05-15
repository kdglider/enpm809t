import numpy as np
import imutils
import cv2

'''
def find_marker(image):
	# convert the image to grayscale, blur it, and detect edges
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 35, 125)
	# find the contours in the edged image and keep the largest one;
	# we'll assume that this is our piece of paper in the image
	cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	c = max(cnts, key = cv2.contourArea)
	# compute the bounding box of the of the paper region and return it
	return cv2.minAreaRect(c)

def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth

def getFocalLength():

	# initialize the known distance from the camera to the object, which
	# in this case is 24 inches
	KNOWN_DISTANCE = 24.0
	# initialize the known object width, which in this case, the piece of
	# paper is 12 inches wide
	KNOWN_WIDTH = 11.0
	# load the furst image that contains an object that is KNOWN TO BE 2 feet
	# from our camera, then find the paper marker in the image, and initialize
	# the focal length
	image = cv2.imread("images/2ft.png")
	marker = find_marker(image)
	focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
	return focalLength
'''


if __name__ == '__main__':
	## Raspberry Pi Camera v2 Specifications
	# Resolution: 	3280 × 2464
	# Sensor Size: 	3.68 x 2.76 mm
	# Focal Length: 3.04 mm

	sensorSizeY = 2.76	# mm
	sensorResY = 2464	# px
	focalLength = 3.04	# mm
	pixPerMil = sensorResY / sensorSizeY	# px/mm
	objectHeight = 2.5	# cm

	imagePath = '../../image4.jpg'

	minHSV = np.array([100, 152, 0])
	maxHSV = np.array([125, 255, 255])

	image = cv2.imread(imagePath)
	imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	blackMask = cv2.inRange(imageHSV, minHSV, maxHSV)

	thresh = cv2.erode(blackMask, None, iterations=3)
	thresh = cv2.dilate(thresh, None, iterations=1)
	ret, thresh = cv2.threshold(thresh, 230, 255, cv2.THRESH_BINARY)

	x, y, w, h = cv2.boundingRect(thresh)

	# Draw contours around detected object
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	cX_object = 0
	cY_object = 0
	cX_frame = int(640/2) # verify if this needs switching
	cY_frame = int(480/2) # verify if this needs switching

	for c in cnts:
		# compute the center of the contour
		M = cv2.moments(c)
		cX_object = int(M["m10"] / M["m00"])
		cY_object = int(M["m01"] / M["m00"])
		# draw the contour and center of the shape on the image
		cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
		cv2.circle(image, (cX_object, cY_object), 7, (255, 255, 255), -1)

	print('Distance: ' + str(objectHeight * focalLength * pixPerMil / h) + 'cm')

	cv2.imshow("image", image)
	cv2.waitKey(0)



