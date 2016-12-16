#!/usr/bin/python

from std_msgs.msg import String
from config import Config
from motor import Motor
from encoder import Encoder
import RPi.GPIO as GPIO
import rospy
import json

class MotorsDriver:
    def __init__(self):
        rospy.init_node("rvr_motors")
        
        if self._set_pins() == False:
            rospy.logerr("Setting pins for motors failed")
            return
        
        self._sub = rospy.Subscriber(
            "rvr_motors",
            String,
            self._motors_callback)
        
        rospy.on_shutdown(self._shutdown_callback)

    def _set_pins(self):
        self.config = Config()
        if self.config == None:
            rospy.logerr("Get config failed")
            return False

        GPIO.setmode(GPIO.BCM)
        self._right = self._init_motor("in1", "in2", "ena")
        self._left = self._init_motor("in3", "in4", "enb")
        self._left_encoder = self._init_encoder("left")
        self._right_encoder = self._init_encoder("right")
        if self._right == None or self._left == None \
            or self._left_encoder == None or self._right_encoder == None:
            return False

        return True

    def _init_motor(self, pin1_s, pin2_s, pinE_s):
        pin1 = self.config.get("motors", pin1_s)
        pin2 = self.config.get("motors", pin2_s)
        pinE = self.config.get("motors", pinE_s)
        if pin1 == None or pin2 == None or pinE == None: 
            rospy.logerr("Get motor pins failed")
            return None

        return Motor(pin1, pin2, pinE)

    def _init_encoder(self, side):
        pin = self.config.get("encoders", side)
        if pin == None:
            rospy.logerr("Get encoder pin failed")
            return None

        return Encoder(pin)

    def _shutdown_callback(self):
        self.stop()
        GPIO.cleanup()

    def _motors_callback(self, msg):
        data = json.loads(msg.data)

        percent_power = data["y"]
        left_power = abs(percent_power) if percent_power is not None else 1
        right_power = abs(percent_power) if percent_power is not None else 1
        
        power_ratio = data["x"]
        if power_ratio is not None and power_ratio > 5:
            right_power = right_power * (1 - power_ratio / 100) 
        elif power_ratio is not None and power_ratio < -5:
            left_power = left_power * (1 - (power_ratio * -1) / 100)

        left_power = left_power if left_power > 20 else 0
        right_power = right_power if right_power > 20 else 0

        left_power = left_power if left_power < 100 else 99
        right_power = right_power if right_power < 100 else 99

        is_forward = percent_power >= 0 if percent_power is not None else True
        self._left.update(left_power, is_forward)
        self._right.update(right_power, is_forward)

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
