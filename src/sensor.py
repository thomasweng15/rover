#!/usr/bin/python

from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
import json
import rospy
import sys

TRIG = 23
ECHO = 24
READ_RATE = 10
PULSE_RATE = 10000

class Sensor:
	def __init__(self):
		rospy.init_node("rvr_ultrasonic", disable_signals=True)	
		self._pub = rospy.Publisher("rvr_ultrasonic", String)

		self.read_rate = rospy.Rate(READ_RATE)
		self.pulse_rate = rospy.Rate(PULSE_RATE)

		GPIO.setmode(GPIO.BCM)
		GPIO.setup(TRIG, GPIO.OUT)
		GPIO.setup(ECHO, GPIO.IN)
		GPIO.output(TRIG, False)

		rospy.loginfo("Waiting for Sensor to Settle")
		time.sleep(2)

	def run(self):
		while not rospy.is_shutdown():
			try:
				GPIO.output(TRIG, True)
				self.pulse_rate.sleep()
				GPIO.output(TRIG, False)

				while GPIO.input(ECHO) == 0:
					pulse_start = time.time()
	
				while GPIO.input(ECHO) == 1:
					pulse_end = time.time()

				pulse_duration = pulse_end - pulse_start
				distance = pulse_duration * 17150
				distance = round(distance, 2)

				self._publish(distance)	
				self.read_rate.sleep()

			except KeyboardInterrupt:
				self._cleanup()
				sys.exit(1)

			except Exception as e:
				rospy.logerr(e)
				self._cleanup()
				sys.exit(1)

	def _publish(self, distance):
		data = { "distance" : distance }
		data_json = json.dumps(data)
		self._pub.publish(String(data_json))

	def _cleanup(self):
		GPIO.cleanup()

if __name__ == '__main__':
	s = Sensor()
	s.run()
