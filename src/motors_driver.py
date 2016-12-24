#!/usr/bin/python

from std_msgs.msg import String
from nav_msgs.msg import Odometry
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
        
        self._sub = rospy.Subscriber(
            "rvr_motors",
            String,
            self._motors_callback)
        
        self.pub = rospy.Publisher(
            "odom",
            Odometry,
            queue_size=50)

        rospy.on_shutdown(self._shutdown_callback)

    def _set_pins(self):
        self.config = Config()
        if self.config == None:
            rospy.logerr("Get config failed")
            return False

        GPIO.setmode(GPIO.BCM)
        self._right = self._init_motor("in1", "in2", "ena", "right")
        self._left = self._init_motor("in3", "in4", "enb", "left")
        return self._right != None and self._left != None

    def _init_motor(self, pin1_s, pin2_s, pinE_s, pin_enc_s):
        pin1 = self.config.get("motors", pin1_s)
        pin2 = self.config.get("motors", pin2_s)
        pinE = self.config.get("motors", pinE_s)
        pinS = self.config.get("encoders", pin_enc_s)
        if pin1 == None or pin2 == None or pinE == None: 
            rospy.logerr("Get motor pins failed")
            return None
    
        if pinS == None:
            rospy.logerr("Get %s encoder pin failed", pin_enc_s)
            return None

        return Motor(pin1, pin2, pinE, pinS)

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
        rate = rospy.Rate(20)
        current_time = rospy.Time.now()
        last_time = current_time
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            odom.child_frame_id = "base_link"

            self.pub.publish(odom)
            rate.sleep()
        
if __name__ == "__main__":
    m = MotorsDriver()
    m.run()
