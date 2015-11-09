#!/usr/bin/python

import json
import rospy
from std_msgs.msg import String

class RoverCtrl:
	def __init__(self, threshold):
		rospy.init_node("rover_controller")
		self.sub = rospy.Subscriber("xboxdrv", String, self.xboxdrv_callback)
		self.threshold = threshold
		self.Y1 = 0
		self.Y2 = 0

	def xboxdrv_callback(self, data):
		datadict = json.loads(data.data)
		self.Y1 = int(datadict["Y1"])
		self.Y2 = int(datadict["Y2"])
		if abs(self.Y1) - self.threshold > 0:
			print "left motor: " + str(self.Y1)
		elif abs(self.Y2) - self.threshold > 0:
			print "right motor: " + str(self.Y2)

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	rover = RoverCtrl(10000)
	rover.run()
