'''
Copyright (c) 2020 Hao Da (Kevin) Dong
@file       hw1.py
@date       2020/02/01
@brief      Moving average implementation and visualization for accelerometer data
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


# Load raw IMU pitch data from 5th column of text file
rawPitch = np.loadtxt("imudata.txt", usecols=4)

# Compute 2, 4, 8, 16, 64, 128 pt moving averages of the raw pitch
pitch2pt = movingAverage(rawPitch, 2)
pitch4pt = movingAverage(rawPitch, 4)
pitch8pt = movingAverage(rawPitch, 8)
pitch16pt = movingAverage(rawPitch, 16)
pitch64pt = movingAverage(rawPitch, 64)
pitch128pt = movingAverage(rawPitch, 128)

# Package all the processed pitch arrays into a list
filteredPitchList = [pitch2pt, pitch4pt, pitch8pt, pitch16pt, pitch64pt, pitch128pt]

# List of window sizes used
windowSizeList = ["2", "4", "8", "16", "64", "128"]

# Create figures and plot all pitch arrays, their means and standard deviations
for i in range(len(filteredPitchList)):
    # Get filtered pitch array from list and calculate mean and standard deviation
    filteredPitch = filteredPitchList[i]
    mean = np.mean(filteredPitch)
    stdev = np.std(filteredPitch)

    # Create x-values to plot pitch against
    xValues = np.arange(0, np.size(filteredPitch), 1)

    # Create arrays out of mean and standard deviation values
    meanArray = mean * np.ones(np.size(filteredPitch))
    upperStdArray = (mean + stdev) * np.ones(np.size(filteredPitch))
    lowerStdArray = (mean - stdev) * np.ones(np.size(filteredPitch))

    # Create a figure and plot rawPitch (truncating trailing values to the length matches),
    # filteredPitch, its mean and its standard deviation
    plt.figure()
    plt.plot(xValues, rawPitch[0:np.size(filteredPitch)], c='r', label='Raw Pitch')
    plt.plot(xValues, filteredPitch, c='g', label='Filtered Pitch')
    plt.plot(xValues, meanArray, c='b', label='Filtered Pitch Mean')
    plt.plot(xValues, upperStdArray, c='b', ls='--', label='Standard Deviation')
    plt.plot(xValues, lowerStdArray, c='b', ls='--')

    # Add title, axis labels and legend
    plt.title(windowSizeList[i] + '-pt Moving Average Filtered Pitch Data')
    plt.xlabel('Pitch Array Element Number')
    plt.ylabel('Pitch Angle (degrees)')
    plt.legend()

# Display all plots
plt.show()




