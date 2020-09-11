import struct
from binascii import hexlify
from codecs import encode
from time import sleep
import serial
from math import pi

class MotorControl():
	def __init__(self, motorId, ser):
		self.motorId = motorId
		# self.ser = super_serial.SerialPort("/dev/ttyUSB0", 9600)
		self.ser = ser 
		self.receive_adress = 0
		self.motor_speed = 0
		self.sync_async = 0
		self.turn_counts_overflow = 0
		self.moving_params = 0
		self.direction_of_turn = 0
		self.turns = 0
	def control_sum_from_one(self, inData, seed):
		for bitsleft in range(8,0,-1):
			temp = ((seed^inData)&0x01)
			if(temp == 0):
				seed = seed >> 1
			else:
				seed = seed ^ 0x18
				seed = seed >> 1
				seed = seed | 0x80
			inData = inData >> 1
		return seed
	def control_sum(self,adress, command, data):
		crc1 = self.control_sum_from_one(adress, 0)
		crc2 = self.control_sum_from_one(command, crc1)
		crc3 = self.control_sum_from_one(data, crc2)
		return crc3

	def set_and_get_response(self, data):
		self.ser.write(data)
		b = self.ser.read(5)
		# print hexlify(b)
		# print len(b)
		return b
	def set_adress(self):
		crc = self.control_sum(0xFF, 0xA0, self.motorId)
		b = bytearray([0xE6, 0xFF, 0xA0, self.motorId, crc])
		response = self.set_and_get_response(b)
		# print hexlify(response)
	def turn_on(self):
		crc = self.control_sum(self.motorId, 0x51, 0x00)
		b = bytearray([0xE6, self.motorId, 0x51, 0, crc])
		response = self.set_and_get_response(b)
		# print hexlify(response)
	def set_speed(self, speed):
		crc = self.control_sum(self.motorId, 0xA3, speed)
		b = bytearray([0xE6, self.motorId, 0xA3, speed, crc])
		response = self.set_and_get_response(b)
		# print hexlify(response)
	
	def get_status(self):
		crc1 = self.control_sum_from_one(self.motorId, 0)
		crc2 = self.control_sum_from_one(0x50,crc1)
		b = bytearray([0xE6, self.motorId, 0x50, crc2])
		response = self.set_and_get_response(b)
		b = bytearray(response)
		# if len(b)>0:
		self.receive_adress = int(b[0])
		speed = float(b[3]) 
	
		self.motor_speed = (speed/60)*(2*pi)
	
		s = '{0:8b}'.format(b[1])
		s = normalize_bit(s)
		self.sync_async = s[0]
		self.turn_counts_overflow = s[1]
		self.moving_params = s[2]
		self.direction_of_turn = s[3]
		turns_highest = s[4:]
		turns_lowest = '{0:8b}'.format(b[2])
		turns = turns_highest + turns_lowest
		turns = normalize_bit(turns)
		self.turns = int(turns, 2)
		
		
	def sef_acceleration(self, data):
		crc = self.control_sum(self.motorId, 0xA5, data)
		b = bytearray([0xE6, self.motorId, 0xA5, data, crc])
		response = self.set_and_get_response(b)
		# print hexlify(response)
	def set_brake(self, data):
		crc = self.control_sum(self.motorId, 0xA6, data)
		b = bytearray([0xE6, self.motorId, 0xA6, data, crc])
		response = self.set_and_get_response(b)
		# print hexlify(response)
	def set_direction(self, data):
		crc = self.control_sum(self.motorId, 0xA7, data)
		b = bytearray([0xE6, self.motorId, 0xA7, data, crc])
		response = self.set_and_get_response(b)
		# print hexlify(response)
	def holl_impulse(self,count_of_impulse):
		crc = self.control_sum(self.motorId, 0xA2, count_of_impulse)
		b = bytearray([0xE6, self.motorId, 0xA2, count_of_impulse, crc])
		response = self.set_and_get_response(b)
	def goal_velocity(self, rps):
		rpm = int((abs(rps)/(2*pi))*60)
		if rps < 0:
			direction = 0
		else:
			direction = 1
		self.set_direction(direction)
		# print(rpm)
		self.set_speed(rpm)

def normalize_bit(bit_string):
	answer = ""
	for i in range(0,len(bit_string)):
		if bit_string[i] == '1' or bit_string[i] == '0':
			answer += bit_string[i]
		else:
			answer += '0'
	return answer