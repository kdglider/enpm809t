'''
Copyright (c) 2020 Hao Da (Kevin) Dong
@file       dataAnalysis.py
@date       2020/03/20
@brief      Creates historgrams and CMA plots from the camera test data
@license    This project is released under the BSD-3-Clause license.
'''

import numpy as np
import matplotlib.pyplot as plt

# Load frame processing times and framerate CMA data
data = np.loadtxt('VideoStreamWebCamTest_Data.txt')

# Get arrays of processing times (in milliseconds) and framerates
processingTimes = 1000 * data[:,0]
framerates = data[:,1]

# Create x-values to plot against
xValues = np.arange(0, np.size(processingTimes), 1)

fig = plt.figure()

# Create processing time histogram
sp2 = fig.add_subplot(2,1,1)
sp2.hist(processingTimes, bins=30, edgecolor='black')
sp2.set_title('Camera Frame Processing Time Histogram')
sp2.set_xlabel('Processing Time (ms)')
sp2.set_ylabel('Number of Frames')

# Create framerate CMA plot
sp1 = fig.add_subplot(2,1,2)
sp1.plot(xValues, framerates, c='r')
sp1.set_title('Camera Framerate Cumulative Moving Average')
sp1.set_xlabel('Frame Number')
sp1.set_ylabel('Framerate CMA (fps)')

# Display all plots
plt.show()




