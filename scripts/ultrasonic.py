#!/usr/bin/python

from std_msgs.msg import String
from config import Config
import RPi.GPIO as GPIO
import time
import json
import rospy
import sys

READ_RATE = 10
PULSE_RATE = 10000

class Ultrasonic:
    def __init__(self):
        rospy.init_node("rvr_ultrasonic", disable_signals=True) 
        self._pub = rospy.Publisher("rvr_ultrasonic", String, queue_size=50)

        self.read_rate = rospy.Rate(READ_RATE)
        self.pulse_rate = rospy.Rate(PULSE_RATE)

        if self._set_pins() == False:
            rospy.logerr("Setting pins for ultrasonic sensor failed")
            return
        
        rospy.on_shutdown(self._shutdown_callback)

        rospy.loginfo("Waiting for ultrasonic sensor to settle...")
        time.sleep(2)
        rospy.loginfo("...done")
    
    def _set_pins(self):
        self.config = Config()
        if self.config == None:
            rospy.logerr("Get config failed")
            return False

        self.trig = self.config.get("ultrasonic", "trig")
        self.echo = self.config.get("ultrasonic", "echo")
        if self.trig == None or self.echo == None:
            rospy.logerr("Get ultrasonic pins failed")
            return False

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)

    def _shutdown_callback(self):
        GPIO.cleanup()

    def run(self):
        while not rospy.is_shutdown():
            try:
                GPIO.output(self.trig, True)
                self.pulse_rate.sleep()
                GPIO.output(self.trig, False)

                while GPIO.input(self.echo) == 0:
                    pulse_start = time.time()
    
                while GPIO.input(self.echo) == 1:
                    pulse_end = time.time()

                pulse_duration = pulse_end - pulse_start
                distance = pulse_duration * 17150
                distance = round(distance, 2)

                self._publish(distance) 
                self.read_rate.sleep()

            except KeyboardInterrupt:
                sys.exit(0) 

    def _publish(self, distance):
        data = { "distance" : distance }
        data_json = json.dumps(data)
        self._pub.publish(String(data_json))

if __name__ == '__main__':
    s = Ultrasonic()
    s.run()
