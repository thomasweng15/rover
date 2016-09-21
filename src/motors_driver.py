#!/usr/bin/python

from std_msgs.msg import String
import RPi.GPIO as GPIO
from motor import Motor
import rospy
import json

DIRECTIONS = set(['stop', 'forward', 'backward', 'right', 'left'])

class MotorsDriver:
	def __init__(self):
		rospy.init_node("rvr_motors")

		GPIO.setmode(GPIO.BCM)
		self._left = Motor(5, 6)
		self._right = Motor(27, 22)

		self._sub = rospy.Subscriber(
			"rvr_motors",
			String,
			self._motors_callback
		)

		rospy.on_shutdown(self._shutdown_callback)

	def _shutdown_callback(self):
		self.stop()
		GPIO.cleanup()

	def _motors_callback(self, msg):
		data = json.loads(msg.data)
		direction = data["direction"]
		assert direction in DIRECTIONS

		if direction == "stop":
			self.stop()
		elif direction == "forward":
			self.move_forward()
		elif direction == "backward":
			self.move_backward()
		elif direction == "left":
			self.turn_left()
		elif direction == "right":
			self.turn_right()

		duration = float(data["duration"])
		rospy.sleep(duration)
		self.stop()
		
	def stop(self):
		self._left.stop()
		self._right.stop()

	def move_forward(self):
		self._left.move_forward()
		self._right.move_forward()

	def move_backward(self):
		self._left.move_backward()
		self._right.move_backward()

	def turn_right(self):
		self._left.move_forward()
		self._right.move_backward()

	def run(self):
		rospy.spin()

if __name__ == "__main__":
	m = MotorsDriver()
	m.run()

