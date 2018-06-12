#!/usr/bin/python

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
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
        self._pub_vel_l = rospy.Publisher("cmd_vel_l", Float64, queue_size=10)
        self._pub_vel_r = rospy.Publisher("cmd_vel_r", Float64, queue_size=10)
        
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

    def velocity_to_power(self, v):
        if v == 0:
            return 0

        min_v = 0
        max_v = 0.35
        clamped_v = max(min_v, min(abs(v), max_v))
        power = 40.2 + 35*clamped_v + 405*(clamped_v**2)
        return min(power, 100)

    def _telop_callback(self, msg):
        """
        Move forward at constant speed.
        If angular velocity is present, turn at constant speed instead.
        """
        x = msg.linear.x
        th = msg.angular.z

        vel_l = 0
        vel_r = 0
        if th == 0:
            vel_l = x
            vel_r = x
        elif th != 0:
            vel_l = th
            vel_r = -th

        self._pub_vel_l.publish(Float64(vel_l))
        self._pub_vel_r.publish(Float64(vel_r))
        
        power_l = self.velocity_to_power(vel_l)
        power_r = self.velocity_to_power(vel_r)
        self._left.update(power_l, vel_l >= 0)
        self._right.update(power_r, vel_r >= 0)

    def stop(self):
        self._left.stop()
        self._right.stop()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    m = MotorsDriver()
    m.run()
