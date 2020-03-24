'''
Copyright (c) 2020 Hao Da (Kevin) Dong
@file       hw3.py
@date       2020/02/20
@brief      Moving average implementation and visualization for accelerometer data
@license    This project is released under the BSD-3-Clause license.
'''

import numpy as np
import cv2
import imutils

image = cv2.imread('dark.jpg')
image = imutils.resize(image, width=500)
imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


desiredHSV = np.array([70, 160, 105])
thresholds = np.array([25, 30, 45])
  
minHSV = desiredHSV - thresholds
maxHSV = desiredHSV + thresholds
 
greenMask = cv2.inRange(imageHSV, minHSV, maxHSV)
maskGray = cv2.cvtColor(greenMask, cv2.COLOR_GRAY2BGR)

filteredImage = cv2.bitwise_and(image, image, mask = greenMask)

# stackedImage = np.hstack((image, imageHSV, maskGray, filteredImage))

x, y, w, h = cv2.boundingRect(greenMask)
cv2.rectangle(image, (x, y), (x+w, y+h), color=(0,0,255), thickness=2)

cv2.imshow('Detect Green', image)
# cv2.imshow('Thresholding Result', stackedImage)


cv2.waitKey(0)


# cv2.imwrite('Thresholding Result.png', stackedImage)






