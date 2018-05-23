#!/usr/bin/python

import RPi.GPIO as GPIO
from config import Config
from std_msgs.msg import Float64
import rospy
import json
import sys

RATE=20 # num per second
DURATION=0.2 # seconds
METERS_PER_TICK=0.3175

class Encoder:
    def __init__(self, enc_id):
        rospy.init_node("encoder_" + enc_id)

        GPIO.setmode(GPIO.BCM)
        self.pin = self._init_pin(enc_id)
        GPIO.setup(self.pin, GPIO.IN)

        self.pin_val = 0

        self.curr_time = rospy.Time.now()
        self.prev_time = self.curr_time
        self.duration = rospy.Duration.from_sec(DURATION)

        self.is_moving_forward = True
        self.forward_ticks = 0
        self.backward_ticks = 0
        self.meters_per_tick = METERS_PER_TICK
        self.meters_per_sec = 0.0

        self.pub = rospy.Publisher(enc_id + "_m/s", Float64, queue_size=50)
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
        self.prev_time = self.curr_time
        self.curr_time = rospy.Time.now()

        new_pin_val = GPIO.input(self.pin)
        tick_detected = self.pin_val == 0 and new_pin_val == 1
        self.pin_val = new_pin_val

        return tick_detected

    def _add_tick_to_queue(self):
        if self.is_moving_forward: 
            self.forward_ticks.append(self.curr_time)
        else:
            self.backward_ticks.append(self.curr_time)

    def _clean_queue(self):
        expiration = self.curr_time - self.duration
        self._remove_expired_ticks(self.forward_ticks, expiration)
        self._remove_expired_ticks(self.backward_ticks, expiration)

    def _remove_expired_ticks(queue, expiration):
        index = None
        for i, ts in enumerate(queue):
            if expiration < ts:
                index = i
                break
        
        if index != None:
            del queue[i:]

    def _calculate_velocity(self):
        total_ticks = len(self.forward_ticks) - len(self.backward_ticks)
        distance = total_ticks * self.meters_per_tick
        self.meters_per_sec = distance * (1.0 / DURATION)

    def _publish_velocity(self):
        msg = Float64()
        msg.data = self.meters_per_sec
        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            rospy.spinOnce()
            
            tick_detected = self._check_for_tick():
            if tick_detected:
                self._add_tick_to_queue()

            self._clean_queue()

            self._calculate_velocity()
            self._publish_velocity()

            rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "No id argument"
        sys.exit(1)
    
    encoder_id = sys.argv[1]
    encoder = Encoder(encoder_id)
    encoder.run()
