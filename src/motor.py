import RPi.GPIO as GPIO
from encoder import Encoder

class Motor:
    def __init__(self, pin_1, pin_2, pin_pwm, pin_enc):
        self.encoder = Encoder(pin_enc)

        GPIO.setmode(GPIO.BCM)
        self.pin_1 = pin_1
        self.pin_2 = pin_2

        GPIO.setup(self.pin_1, GPIO.OUT)
        GPIO.setup(self.pin_2, GPIO.OUT)
        GPIO.output(self.pin_1, False)
        GPIO.output(self.pin_2, False)

        GPIO.setup(pin_pwm, GPIO.OUT)
        self.pwm = GPIO.PWM(pin_pwm, 100)
        self.pwm.start(50)

    def update(self, percent, is_forward):
        self.pwm.ChangeDutyCycle(percent)
        if is_forward:
            self.move_forward()
        else:
            self.move_backward()

    def update_speed(self, percent):
        self.pwm.ChangeDutyCycle(percent)

    def move_forward(self):
        GPIO.output(self.pin_1, True)
        GPIO.output(self.pin_2, False)

    def move_backward(self):
        GPIO.output(self.pin_1, False)
        GPIO.output(self.pin_2, True)

    def stop(self):
        GPIO.output(self.pin_1, False)
        GPIO.output(self.pin_2, False)
        self.pwm.stop()
