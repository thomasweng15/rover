import RPi.GPIO as GPIO

class Motor:
	def __init__(self, pin_1, pin_2):
		self.pin_1 = pin_1
		self.pin_2 = pin_2

		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.pin_1, GPIO.OUT)
		GPIO.setup(self.pin_2, GPIO.OUT)
		GPIO.output(self.pin_1, False)
		GPIO.output(self.pin_2, False)

	def move_forward(self):
		GPIO.output(self.pin_1, True)
		GPIO.output(self.pin_2, False)

	def move_backward(self):
		GPIO.output(self.pin_1, False)
		GPIO.output(self.pin_2, True)

	def stop(self):
		GPIO.output(self.pin_1, False)
		GPIO.output(self.pin_2, False)
