#!/usr/bin/python

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from config import Config
from motor import Motor
import RPi.GPIO as GPIO
import rospy
import json

class MotorsDriver:
    def __init__(self):
        rospy.init_node("rvr_motors")
        
        if self._set_pins() == False:
            rospy.logerr("Setting pins for motors failed")
            return
        
        self._sub = rospy.Subscriber("cmd_vel", Twist, self._telop_callback)
        rospy.on_shutdown(self._shutdown_callback)

    def _set_pins(self):
        self.config = Config()
        if self.config == None:
            rospy.logerr("Get config failed")
            return False

        GPIO.setmode(GPIO.BCM)
        self._right = self._init_motor("in1", "in2", "ena")
        self._left = self._init_motor("in3", "in4", "enb")
        return self._right != None and self._left != None

    def _init_motor(self, pin1_s, pin2_s, pinE_s):
        pin1 = self.config.get("motors", pin1_s)
        pin2 = self.config.get("motors", pin2_s)
        pinE = self.config.get("motors", pinE_s)
        if pin1 == None or pin2 == None or pinE == None: 
            rospy.logerr("Get motor pins failed")
            return None
    
        return Motor(pin1, pin2, pinE)

    def _shutdown_callback(self):
        self.stop()
        GPIO.cleanup()

    def _telop_callback(self, msg):
        linear = msg.linear

        self._left.update(abs(linear.x*100), linear.x > 0)
        self._right.update(abs(linear.x*100), linear.x > 0)

    def stop(self):
        self._left.stop()
        self._right.stop()
        
    def update_speed(self, percent):
        if percent > 100:
            rospy.logwarn("speed percentage out of bounds: %s", str(percent) + "%")
            return 
        
        if percent < 20:
            percent = 0

        self._left.update_speed(percent)
        self._right.update_speed(percent)
        
    def run(self):
        rospy.spin()
        
if __name__ == "__main__":
    m = MotorsDriver()
    m.run()
