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
        self._left = Motor(5, 6, 26)
        self._right = Motor(27, 22, 16)
        self._sub = rospy.Subscriber(
            "rvr_motors",
            String,
            self._motors_callback)
        
        rospy.on_shutdown(self._shutdown_callback)

    def _shutdown_callback(self):
        self.stop()
        GPIO.cleanup()

    def _motors_callback(self, msg):
        data = json.loads(msg.data)

        percent_power = data["y"]
        left_power = abs(percent_power) if percent_power is not None else 1
        right_power = abs(percent_power) if percent_power is not None else 1
        
        power_ratio = data["x"]
        if power_ratio is not None and power_ratio > 5:
            right_power = right_power * (1 - power_ratio / 100) 
        elif power_ratio is not None and power_ratio < -5:
            left_power = left_power * (1 - (power_ratio * -1) / 100)

        left_power = left_power if left_power > 20 else 0
        right_power = right_power if right_power > 20 else 0

        left_power = left_power if left_power < 100 else 99
        right_power = right_power if right_power < 100 else 99

        is_forward = percent_power >= 0 if percent_power is not None else True
        self._left.update(left_power, is_forward)
        self._right.update(right_power, is_forward)

    def _motors_callbackq(self, msg):
        data = json.loads(msg.data)
        rospy.loginfo(data)
        
        speed = data["percent"]
        if speed is not None:
            self.update_speed(speed)
            
        direction = data["direction"]
        assert direction in DIRECTIONS
        
        if direction == "stop":
            self.stop()
        elif direction == "forward":
            rospy.loginfo("forward")
            self.move_forward()
        elif direction == "backward":
            self.move_backward()
        elif direction == "left":
            self.turn_left()
        elif direction == "right":
            self.turn_right()
            
        duration = float(data["duration"])
        #rospy.sleep(duration)
        #self.stop()

    def stop(self):
        self._left.stop()
        self._right.stop()
        
    def update_speed(self, percent):
        if percent > 100:
            rospy.logwarn("speed percentage out of bounds: %s", str(percent) + "%")
            return 
        
        if percent < 20:
            percent = 0

        self._left.update_speed(percent)
        self._right.update_speed(percent)
        
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
