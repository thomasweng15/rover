#!/usr/bin/python

import RPi.GPIO as GPIO
from config import Config
from std_msgs.msg import Float32, String, Bool
import rospy
import json
import sys

TICK_TO_M=0.1
RATE=1 # 1 per second

class Encoder:
    def __init__(self, encoder_id):
        rospy.init_node("encoder_" + encoder_id)

        GPIO.setmode(GPIO.BCM)
        self.pin = self._init_pin(encoder_id)
        GPIO.setup(self.pin, GPIO.IN)

        self.pin_val = 0

        self.curr_time = rospy.Time.now()
        self.prev_time = self.curr_time

        self.is_moving_forward = True
        self.forward_ticks = 0
        self.backward_ticks = 0
        self.meters_per_sec = 0.0

        self.pub = rospy.Publisher(encoder_id + "_m/s", Float32, queue_size=50)
        self.sub = rospy.Subscriber("wheel_dir_" + encoder_id, Bool, self._wheel_dir_callback)
        rospy.on_shutdown(self._shutdown_callback)

    def _wheel_dir_callback(self, msg):
        rospy.loginfo(msg)
        self.is_moving_forward = msg.data

    def _shutdown_callback(self):
        GPIO.cleanup()

    def _init_pin(self, encoder_id):
        config = Config()
        if config == None: 
            rospy.logerr("Get config failed")
            sys.exit(1) 

        pin =  config.get("encoders", encoder_id) 
        if pin == None:
            rospy.logerr("Get encoder pin failed")
            sys.exit(1)

        return pin

    def _update(self):
        self.prev_time = self.curr_time
        self.curr_time = rospy.Time.now()

        new_pin_val = GPIO.input(self.pin)
        tick_passed = self.pin_val == 0 and new_pin_val == 1
        self.forward_ticks += 1 if tick_passed and self.is_moving_forward else 0
        self.backward_ticks += 1 if tick_passed and not self.is_moving_forward else 0
        self.pin_val = new_pin_val

        self._update_velocity()
        self._publish_velocity()

    def _update_velocity(self):
        # TODO
        self.meters_per_sec = 0

    def _publish_velocity(self):
        msg = Float32()
        msg.data = self.meters_per_sec
        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self._update()
            rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "No id argument"
        sys.exit(1)
    
    encoder_id = sys.argv[1]
    encoder = Encoder(encoder_id)
    encoder.run()
