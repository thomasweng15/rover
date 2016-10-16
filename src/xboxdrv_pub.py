#!/usr/bin/python

import sys
import re
import json
import rospy
from std_msgs.msg import String

class XboxPub():
	def __init__(self):
		rospy.init_node("xboxdrv")
		self.pub = rospy.Publisher("/xboxdrv", String)
		
	def run(self):
		while not rospy.is_shutdown():
			line = sys.stdin.readline().rstrip()
			line = re.sub(r":\s*", ":", line)
			if line.find("LIBUSB_ERROR_BUSY") is not -1:
				rospy.logerr("Error: libusb is locked")

			datalist = line.split(' ')
			datadict = {}
			for item in datalist:
				pair = item.split(":")
				if len(pair) == 2:
					datadict[pair[0]] = pair[1]

			datajson = json.dumps(datadict, separators=(',', ':'))
			self.pub.publish(String(datajson))

if __name__ == '__main__':
	x = XboxPub()
	x.run()
