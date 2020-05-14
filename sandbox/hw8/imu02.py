'''
Copyright (c) 2020 Hao Da (Kevin) Dong, Krithika Govindaraj
@file       imu01.py
@date       2020/05/01
@brief      Read IMU data through a serial stream from an Arduino
@license    This project is released under the BSD-3-Clause license.
'''

import serial
import time

# Create serial connection
ser = serial.Serial('/dev/ttyUSB0', 9600)

# Flush initial readings
time.sleep(5)
ser.reset_input_buffer()

while (True):
    # Check if there is data in the input buffer
	if (ser.in_waiting > 0):
		# Read serial stream
		angle = ser.readline()   
		print(angle)  
		
		# Read serial stream
		line = ser.readline()

		# Strip newline and return carriage from line
		line = line.rstrip().lstrip()

		# Convert line to string, strip non-numeric characters and convert to float
		line = str(line)
		line = line.strip("'").strip("b'")
		print(line)
		angle = float(line)
		print(str(angle) + '\n')




