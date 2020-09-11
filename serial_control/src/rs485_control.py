#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import struct
from binascii import hexlify
from codecs import encode
from time import sleep
import serial
from math import pi
from MotorControl import MotorControl


if __name__ == '__main__':
	ser = serial.Serial('/dev/ttyUSB0', timeout=1.0)
	motor_1 = MotorControl(0, ser)
	motor_2 = MotorControl(1, ser)
	motor_1.holl_impulse(8)
	motor_1.sef_acceleration(20)
	motor_1.set_brake(20)
	motor_1.set_direction(1)
	motor_1.turn_on()
	motor_2.holl_impulse(8)
	motor_2.sef_acceleration(20)
	motor_2.set_brake(20)
	motor_2.set_direction(1)
	motor_2.turn_on()
	while True:
		# controller.goal_velocity(0.5)
		motor_1.goal_velocity(0)
		motor_2.goal_velocity(0)
		# motor_1.set_speed(0)
		# motor_2.set_speed(0)
		b = motor_1.get_status()
		print("big motor:"+str(motor_1.motor_speed))
		b = motor_2.get_status()
		print("small motor:"+str(motor_2.motor_speed))
		# print controller.direction_of_turn #1 - cw, 0 - ccw
		sleep(0.01)

	# controller.set_adress()