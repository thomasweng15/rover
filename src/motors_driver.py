#!/usr/bin/python

import RPi.GPIO as GPIO
from motor import Motor
import time

class MotorsDriver:
	def __init__(self):
		self._left = Motor(5, 6)
		self._right = Motor(27, 22)

	def stop(self):
		self._left.stop()
		self._right.stop()

	def move_forward(self, duration):
		self._left.move_forward()
		self._right.move_forward()
		time.sleep(duration)

	def move_backward(self, duration):
		self._left.move_backward()
		self._right.move_backward()
		time.sleep(duration)

	def turn_right(self, duration):
		self._left.move_forward()
		self._right.move_backward()
		time.sleep(duration)

if __name__ == "__main__":
	m = MotorsDriver()
	m.turn_right(0.30)
	m.stop()
	GPIO.cleanup()

