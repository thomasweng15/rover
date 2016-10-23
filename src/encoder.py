import RPi.GPIO as GPIO

class Encoder:
    def __init__(self, pin):
        GPIO.setmode(GPIO.BCM)
        self.pin = pin

        GPIO.setup(self.pin, GPIO.IN)
        self.value = 0
        self.prev_value = 0
        self.ticks = 0
        self.rotations = 0 

    def passed_tick(self):
        passed_tick = False
        self.value = GPIO.input(self.pin)
        if self.value == 0 and self.prev_value == 1:
            self.update_ticks()
            passed_tick = True

        self.prev_value = self.value
        return passed_tick

    def update_ticks(self):
        self.ticks += 1
        if self.ticks > 20:
            self.rotations += 1
            self.ticks = 0
            
