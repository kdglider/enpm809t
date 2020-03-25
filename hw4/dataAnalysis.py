'''
Copyright (c) 2020 Hao Da (Kevin) Dong
@file       dataAnalysis.py
@date       2020/02/01
@brief      Creates historgrams and plots from the frame rate data
@license    This project is released under the BSD-3-Clause license.
'''

import numpy as np
import matplotlib.pyplot as plt

# Load processing times (in milliseconds) from text file
processingTimes = 1000 * np.loadtxt("hw4data.txt")

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




