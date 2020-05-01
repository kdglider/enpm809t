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
count = 0
# Flush initial readings
time.sleep(5)
ser.flush()


while (True):
    if (ser.in_waiting > 0):
        
        # Read serial stream
        line = ser.readline()
        print(line)     
        # Strip extra characters from serial data and convert to float
        line = line.rstrip().lstrip().strip("'").strip("b'")
        data = float(line)

        print(data, '\n')



