#!/usr/bin/python

#Display Data from Neato LIDAR
#based on code from Nicolas "Xevel" Saugnier
#requires pyserial

from rover.msg import LidarPoint
from lidar_viz import LidarViz
import rospy
import serial
import sys

COM_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

class Lidar:
	def __init__(self, com_port, baudrate, hasViz):
		rospy.init_node("rvr_lidar")

		self.pub = rospy.Publisher("/lidar", LidarPoint)

		self.lidarData = [LidarPoint() for i in range(360)]
		self.init_level = 0
		self.index = 0
		self.serial = serial.Serial(com_port, baudrate)		
		self.hasViz = hasViz
		
		if self.hasViz:
			self.viz = LidarViz(8000)

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
		return float(data[0] | (data[1] << 8)) / 64.0

	def update_position(self, angle, data):
		# unpack data
		x0 = data[0]
		x1 = data[1]
		x2 = data[2]
		x3 = data[3]

		dist_mm = x0 | ((x1 & 0x3f) << 8) # distance is coded on 13 bits ? 14 bits ?
		quality = x2 | (x3 << 8) # quality is on 16 bits
		self.lidarData[angle].dist_mm = dist_mm
		self.lidarData[angle].quality = quality

		point = LidarPoint()
		point.angle = angle
		point.dist_mm = dist_mm
		point.quality = quality

		self.pub.publish(point)
		
		if self.hasViz:
			self.viz.redraw(x1, angle, dist_mm, quality)

	def run(self):
		while not rospy.is_shutdown():
			try: 
				rospy.sleep(0.00001)
				
				if self.hasViz:
					self.viz.checkKeys()

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
					
						if self.hasViz:
							self.viz.update_speed(int(speed_rpm))
				
						self.update_position(self.index * 4 + 0, b_data0)
						self.update_position(self.index * 4 + 1, b_data1)
						self.update_position(self.index * 4 + 2, b_data2)
						self.update_position(self.index * 4 + 3, b_data3)
					else:
						rospy.logwarn("checksum mismatch")

						null_data = [0, 0x80, 0, 0]
						self.update_position(self.index * 4 + 0, null_data)
						self.update_position(self.index * 4 + 1, null_data)
						self.update_position(self.index * 4 + 2, null_data)
						self.update_position(self.index * 4 + 3, null_data)

					self.init_level = 0
			except Exception as e: 
				rospy.logerr(e)

if __name__ == '__main__':
	hasViz = False
	if len(sys.argv) > 1 and sys.argv[1] == "-v":
		hasViz = True

	l = Lidar(COM_PORT, BAUDRATE, False)
	l.run()
