#!/usr/bin/python

import sys
import re
import json
import rospy
from rover.msg import XboxController

class XboxPub():
    def __init__(self):
        rospy.init_node("xboxdrv")
        self.pub = rospy.Publisher("/xboxdrv", XboxController)
        
    def get_xbox_msg(self, line):
        datalist = line.split(' ')
        datadict = {}
        for item in datalist:
            pair = item.split(":")
            if len(pair) == 2:
                datadict[pair[0]] = int(pair[1])
		
        data = XboxController()
        data.A = False if datadict["A"] == 0 else True
        data.RT = datadict["RT"]
        data.dl = False if datadict["dl"] == 0 else True
        data.X1 = datadict["X1"]
        data.Y = False if datadict["Y"] == 0 else True
        data.start = False if datadict["start"] == 0 else True
        data.dd = False if datadict["dd"] == 0 else True
        data.LB = False if datadict["LB"] == 0 else True
        data.TR = False if datadict["TR"] == 0 else True
        data.back = False if datadict["back"] == 0 else True
        data.LT = datadict["LT"]
        data.X2 = datadict["X2"]
        data.TL = False if datadict["TL"] == 0 else True
        data.B = False if datadict["B"] == 0 else True
        data.RB = False if datadict["RB"] == 0 else True
        data.Y1 = datadict["Y1"]
        data.X = False if datadict["X"] == 0 else True
        data.du = False if datadict["du"] == 0 else True
        data.dr = False if datadict["dr"] == 0 else True
        data.guide = False if datadict["guide"] == 0 else True
        data.Y2 = datadict["Y2"]
            
        return data
        
    def run(self):
        while not rospy.is_shutdown():
            line = sys.stdin.readline().rstrip()
            if line == "": break;
            line = re.sub(r":\s*", ":", line)
            if line.find("LIBUSB_ERROR_BUSY") is not -1:
                rospy.logerr("Error: libusb is locked")
                    
            data = self.get_xbox_msg(line)
            self.pub.publish(data)
                
if __name__ == '__main__':
    p = XboxPub()
    p.run()
