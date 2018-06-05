#!/usr/bin/python

import RPi.GPIO as GPIO
from config import Config
from std_msgs.msg import Float64
from rover.msg import BoolStamped
import rospy
import json
import sys

RATE=200 # num per second

class Encoder:
    def __init__(self, enc_id):
        rospy.init_node("encoder_" + enc_id)

        GPIO.setmode(GPIO.BCM)
        self.pin = self._init_pin(enc_id)
        GPIO.setup(self.pin, GPIO.IN)

        self.pin_val = 0

        self.is_moving_forward = True

        self.pub = rospy.Publisher("encoder_" + enc_id, BoolStamped, queue_size=50)
        self.sub = rospy.Subscriber("cmd_vel_" + enc_id, Float64, self._cmd_vel_cb)
        rospy.on_shutdown(self._shutdown_callback)

    def _cmd_vel_cb(self, msg):
        self.is_moving_forward = msg.data >= 0

    def _shutdown_callback(self):
        GPIO.cleanup()

    def _init_pin(self, enc_id):
        config = Config()
        if config == None: 
            rospy.logerr("Get config failed")
            sys.exit(1) 

        pin = config.get("encoders", enc_id) 
        if pin == None:
            rospy.logerr("Get encoder pin failed")
            sys.exit(1)

        return pin

    def _check_for_tick(self):
        curr_time = rospy.Time.now()

        new_pin_val = GPIO.input(self.pin)
        tick_detected = self.pin_val == 0 and new_pin_val == 1
        self.pin_val = new_pin_val

        return curr_time if tick_detected else None

    def _publish_tick(self, timestamp):
        msg = BoolStamped()
        msg.header.stamp = timestamp
        msg.is_forward.data = self.is_moving_forward
        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            timestamp = self._check_for_tick()
            if timestamp:
                self._publish_tick(timestamp)

            rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "No id argument"
        sys.exit(1)
    
    encoder_id = sys.argv[1]
    encoder = Encoder(encoder_id)
    encoder.run()
