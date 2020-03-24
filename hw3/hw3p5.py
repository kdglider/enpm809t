'''
Copyright (c) 2020 Hao Da (Kevin) Dong
@file       hw3p5.py
@date       2020/02/01
@brief      Visualization of RPi camera frame processing times
@license    This project is released under the BSD-3-Clause license.
'''

import numpy as np
import matplotlib.pyplot as plt

'''
@brief      Applies a moving average to a np array
@param      rawArray    np array
@param      windowSize  Size of the moving average window
@return     newArray    Processed array
'''
def movingAverage(rawArray, windowSize):
    # Create empty new array accounting for the window size
    newArray = np.empty(np.size(rawArray) - (windowSize - 1))

    # Fill up newArray with averaged values from rawArray
    for i in range(np.size(newArray)):
        # Slice a window from rawArray
        window = rawArray[i : i + windowSize]

        # Average the window elements
        newArray[i] = np.mean(window)

    return newArray


# Load processing times (in milliseconds) from text file
processingTimes = 1000 * np.loadtxt("hw3data.txt")

# Create x-values to plot pitch against
xValues = np.arange(0, np.size(processingTimes), 1)

# Create a figure and plot rawPitch (truncating trailing values to the length matches),
# filteredPitch, its mean and its standard deviation
fig = plt.figure()
sp1 = fig.add_subplot(1,2,1)
sp1.plot(xValues, processingTimes, c='r')
sp1.set_title('Raspberry Pi Camera Frame Processing Times')
sp1.set_xlabel('Frame Number')
sp1.set_ylabel('Processing Time (ms)')

sp2 = fig.add_subplot(1,2,2)
sp2.hist(processingTimes, bins=30, edgecolor='black')
sp2.set_title('Processing Time Histogram')
sp2.set_xlabel('Processing Time (ms)')
sp2.set_ylabel('Number of Frames')

# Display all plots
plt.show()




