#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import struct
from binascii import hexlify
from codecs import encode
from time import sleep
import serial
from math import pi
from MotorControl import MotorControl
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
class DifControl():
	def __init__(self, ser):	
		self.motor_r = MotorControl(0, ser)
		self.motor_l = MotorControl(1, ser)
		self.R = 0.115
		self.l = 0.5081
	def set_speed(self, vX, wZ): #meteres per second / radians per second
		v_r = -1*(2*vX - self.l*wZ)/(2*self.R)
		v_l = (2*vX + self.l*wZ)/(2*self.R)
		# print("v_r:"+str(v_r)+" v_l:"+str(v_l))
		self.motor_r.goal_velocity(v_r)
		self.motor_l.goal_velocity(v_l)

	def get_motors_speed(self):
		self.motor_r.get_status()
		self.motor_l.get_status()
		if self.motor_r.direction_of_turn == '0':
			v_r = self.motor_r.motor_speed 
		else:
			v_r = -self.motor_r.motor_speed 
		if self.motor_l.direction_of_turn == '1':
			v_l = self.motor_l.motor_speed 
		else:
			v_l = -self.motor_l.motor_speed
		return v_r, v_l
	def init_motors(self, holl, acc, br):
		self.motor_r.holl_impulse(holl)
		self.motor_r.sef_acceleration(acc)
		self.motor_r.set_brake(br)
		self.motor_r.turn_on()
		
		self.motor_l.holl_impulse(holl)
		self.motor_l.sef_acceleration(acc)
		self.motor_l.set_brake(br)
		self.motor_l.turn_on()

def cb_cmd_vel(data):
	global v_X_targ, w_Z_targ 
	if abs(data.linear.x) > 0.01 and abs(data.linear.x) < 0.09:
		v_X_targ = sign(data.linear.x)*0.1
	else:	
		v_X_targ = data.linear.x
	if abs(data.angular.z) > 0.01 and abs(data.angular.z) < 0.1:
		w_Z_targ = sign(data.angular.z)*0.1
	else:	
		w_Z_targ = data.angular.z

def sign(value):
	if value >= 0:
		return 1
	else:
		return -1
if __name__ == '__main__':
	ser = serial.Serial('/dev/ttyUSB0', timeout=1.0)
	dif = DifControl(ser)
	dif.init_motors(10, 20, 20)
	v_X_targ = 0
	w_Z_targ = 0
	rospy.init_node("differential_control")
	sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, cb_cmd_vel, queue_size = 4)
	joints_states_pub = rospy.Publisher("/joint_states_wheels", JointState, queue_size = 4)
	msg = JointState()
	msg.name.append("right")
	msg.name.append("left")
	msg.velocity.append(0)
	msg.velocity.append(0)
	while not rospy.is_shutdown():
		try:
			dif.set_speed(v_X_targ, w_Z_targ)
			v_r, v_l = dif.get_motors_speed()
			msg.velocity[0] = v_r
			msg.velocity[1] = v_l
			joints_states_pub.publish(msg)
			sleep(0.1)
		except KeyboardInterrupt:
			break
