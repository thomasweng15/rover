#!/usr/bin/python

import sys
import re
import json
import rospy
from std_msgs.msg import String

rospy.init_node("xboxdrv")
pub = rospy.Publisher('xboxdrv', String)

while True:
	line = sys.stdin.readline().rstrip()
	line = re.sub(r":\s*", ":", line)
	if line.find("LIBUSB_ERROR_BUSY") is not -1:
		sys.exit("Error: libusb is locked")

	datalist = line.split(' ')
	datadict = {}
	for item in datalist:
		pair = item.split(":")
		if len(pair) == 2:
			datadict[pair[0]] = pair[1]
	
	datajson = json.dumps(datadict, separators=(',',':'))
	pub.publish(String(datajson))

