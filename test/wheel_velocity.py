#!/usr/bin/python

from config import Config
from motor import Motor
from encoder import Encoder
import Rpi.GPIO as GPIO
import json
import sys
import time

def _init_motor(config, pin1_s, pin2_s, pinE_s):
    pin1 = config.get("motors", pin1_s)
    pin2 = config.get("motors", pin2_s)
    pinE = config.get("motors", pinE_s)
    if pin1 == None or pin2 == None or pinE == None: 
        print "Get motor pins failed"
        return None

    return Motor(pin1, pin2, pinE)

config = Config()
if config == None:
    print "Get config failed"
    sys.exit(1)

GPIO.setmode(GPIO.BCM)
right = _init_motor(config, "in1", "in2", "ena")
right_enc = Encoder("right")

max_power = 40
try:
    for i in range(20, max_power + 1, 10):
        print "Updating power to " + i
        right.update(i, True)

        count = 0
        while count < 5:
            count++;
            time.sleep(1)
except Exception as e:
    print e
    right.stop()

right.stop()