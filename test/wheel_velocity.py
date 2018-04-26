#!/usr/bin/python

import sys
import os
sys.path.append(os.path.abspath('../scripts'))
from config import Config
from motor import Motor
from encoder import Encoder
import RPi.GPIO as GPIO
import json
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
right.update(0, True)
right_enc = Encoder("right")

power = 100
try:
    print "Updating power to " + str(power)
    right.update(power, True)
    
    t_end = time.time() + 10
    while time.time() < t_end:
        x = 1 # stub
except Exception as e:
    print e
    right.stop()

right.stop()
