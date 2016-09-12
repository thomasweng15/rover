#!/usr/bin/python

from motor import Motor
import time

class MotorsDriver:
	def __init__(self):
		self._left = Motor(5, 6)
		self._right = Motor(27, 22)

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
		print "turn right"

if __name__ == "__main__":
	m = MotorsDriver()
	m.move_forward()
	time.sleep(2)
	m.stop()

	m.move_backward()
	time.sleep(2)
	m.stop()

