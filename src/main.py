#!/usr/bin/python

from std_msgs.msg import String
from motors_driver import MotorsDriver
import RPi.GPIO as GPIO
import json
import rospy
import time

DISTANCE_THRESHOLD = 10

class Controller:
	def __init__(self, dist_threshold):
		rospy.init_node("rvr_controller")
		GPIO.setmode(GPIO.BCM)
		self._mode = "manual" 
		self._init_subs()
		self._dist_threshold = dist_threshold
		self._motors = MotorsDriver()
		rospy.on_shutdown(self._shutdown_callback)

	def _init_subs(self):
		self._control_sub = rospy.Subscriber(
			"rvr_mode",
			String,
			self._mode_callback		
		)

		self._ultrasonic_sub = rospy.Subscriber(
			"rvr_ultrasonic", 
			String, 
			self._ultrasonic_callback
		)

	def _mode_callback(self, msg):
		data = json.loads(msg.data)
		mode = data["mode"]		
		if mode == "auto" or mode == "manual":
			self._mode = mode
		
	def _ultrasonic_callback(self, msg):
		if self._mode == "manual":
			return

		data = json.loads(msg.data)
		distance = data["distance"]
		if distance < self._dist_threshold:
			rospy.logerr("close: " + str(distance))
			self._motors.stop()
			rospy.loginfo("backward")
			self._motors.move_backward(0.3)
			time.sleep(0.3)
			self._motors.stop()
			self._motors.turn_right(0.3)
			time.sleep(0.3)
			self._motors.stop()

	def _shutdown_callback(self):
		self._motors.stop()
		GPIO.cleanup()

	def run(self):
		while not rospy.is_shutdown():
			if self._mode == "auto":
				rospy.loginfo("forward")
				self._motors.move_forward(0.3)
			
			time.sleep(0.3)

if __name__ == '__main__':
	rover = Controller(DISTANCE_THRESHOLD)
	rover.run()
