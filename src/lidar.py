#Display Data from Neato LIDAR
#based on code from Nicolas "Xevel" Saugnier
#requires vpython and pyserial

import rospy
import thread 
import sys
import traceback
import math
import serial

COM_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

class Lidar:
	def __init__(self, com_port, baudrate):
		rospy.init_node("rvr_lidar")

		self.data = [[] for i in range(360)]
		self.init_level = 0
		self.index = 0
		self.serial = serial.Serial(com_port, baudrate)		

	def checksum(self, data):
		"""
		Compute and return the checksum as an int.

		data -- list of 20 bytes (as ints), in the order they arrived.

		"""
		# group the data by word, little-endian
		data_list = []
		for t in range(10):
			data_list.append(data[2*t] + (data[2*t+1] << 8))

		# compute the checksum on 32 bits
		chk32 = 0
		for d in data_list:
			chk32 = (chk32 << 1) + d
		
		# return a value wrapped around on 15 bites, and truncated to still fit into 15 bits
		checksum = (chk32 & 0x7FFF) + (chk32 >> 15) # wrap around to fit
		checksum = checksum & 0x7FFF # truncate to 15 bits
		return int(checksum)		

	def compute_speed(self, data):
		speed_rpm = float(data[0] | (data[1] << 8)) / 64.0
		return speed_rpm

	def run(self):
		while not rospy.is_shutdown():
			try: 
				rospy.sleep(0.00001)

				if self.init_level == 0:							
					byte = ord(self.serial.read(1))
					# start byte
					if byte == 0xFA:
						self.init_level = 1
					else:
						self.init_level = 0
				elif self.init_level == 1:
					# position index 
					byte = ord(self.serial.read(1))
					if byte >= 0xA0 and byte <= 0xF9:
						self.index = byte - 0xA0
						self.init_level = 2
					elif byte != 0xFA:
						self.init_level = 0
				elif self.init_level == 2:
					# speed
					b_speed = [ord(b) for b in self.serial.read(2)]

					# data
					b_data0 = [ord(b) for b in self.serial.read(4)]
					b_data1 = [ord(b) for b in self.serial.read(4)]
					b_data2 = [ord(b) for b in self.serial.read(4)]
					b_data3 = [ord(b) for b in self.serial.read(4)]

					# for the checksum, we need all the data of the packet...
					# this could be collected in a more elegant fashion...
					all_data = [0xFA, self.index+0xA0] + b_speed + b_data0 + b_data1 + b_data2 + b_data3

					# checksum
					b_checksum = [ord(b) for b in self.serial.read(2)]
					incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

					# verify that the received checksum is equal to the one computed from the data
					if self.checksum(all_data) == incoming_checksum:
						speed_rpm = self.compute_speed(b_speed)
						rospy.loginfo(speed_rpm)
					else:
						rospy.logwarn("checksum mismatch")

					self.init_level = 0
			except Exception as e: 
				rospy.logerr(e)

if __name__ == '__main__':
	l = Lidar(COM_PORT, BAUDRATE)
	l.run()
