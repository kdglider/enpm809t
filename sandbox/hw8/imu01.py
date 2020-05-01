'''
Copyright (c) 2020 Hao Da (Kevin) Dong, Krithika Govindaraj
@file       imu01.py
@date       2020/05/01
@brief      Read IMU data through a serial stream from an Arduino
@license    This project is released under the BSD-3-Clause license.
'''

import serial

# Create serial connection
ser = serial.Serial('/dev/ttyUSB0', 9600)

count = 0

while (True):
    if (ser.in_waiting > 0):
        count += 1

        # Read serial stream
        line = ser.readline()
        print(line)

        # Discard the first 10 lines of data
        if (count > 10):
            # Strip newline and return carriage from line
            line = line.rstrip().lstrip()

            # Convert line to string, strip non-numeric characters and convert to float
            line = str(line)
            line = line.strip("'").strip("b'")
            data = float(line)

            print(data, '\n')



