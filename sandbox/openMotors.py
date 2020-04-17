'''
Copyright (c) 2020 Hao Da (Kevin) Dong, Krithika Govindaraj
@file       openMotors.py
@date       2020/04/17
@brief      Script to run at RPi startup to set all H-bridge motor control pins low
@license    This project is released under the BSD-3-Clause license.
'''

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(31, GPIO.OUT)        # IN1
GPIO.setup(33, GPIO.OUT)        # IN2
GPIO.setup(35, GPIO.OUT)        # IN3
GPIO.setup(37, GPIO.OUT)        # IN4

GPIO.output(31, False)
GPIO.output(33, False)
GPIO.output(35, False)
GPIO.output(37, False)

GPIO.cleanup()
