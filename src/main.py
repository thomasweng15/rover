#!/usr/bin/python

from std_msgs.msg import String
from rover.msg import XboxController 
from motors_driver import MotorsDriver
import json
import rospy

RATE = 0.001
DISTANCE_THRESHOLD = 10

class Controller:
    def __init__(self, dist_threshold):
        rospy.init_node("rvr_controller")
        self._is_auto = False 
        self._x_pressed = False 
        self._dist_threshold = dist_threshold
        self._is_avoiding_collision = False
        
        self._init_subs()
        self._pub = rospy.Publisher("rvr_motors", String)

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
                
        self._xbox_sub = rospy.Subscriber(
            "xboxdrv", 
            XboxController, 
            self._xbox_callback
        )

    def _mode_callback(self, msg):
        data = json.loads(msg.data)
        mode = data["mode"]     
        if mode == "auto" or mode == "manual":
            self._is_auto = not self._is_auto
        
    def _ultrasonic_callback(self, msg):
        if not self._is_auto or self._is_avoiding_collision:
            return

        data = json.loads(msg.data)
        distance = data["distance"]
        if distance < self._dist_threshold:
            rospy.logwarn("Detect close object: " + str(distance))
            self._start_collision_avoidance()

    def _xbox_callback(self, msg):
        if msg.X == True:
            self._x_pressed = True
        elif msg.X == False and self._x_pressed:
            self._x_pressed = False
            self._is_auto = not self._is_auto
        
        if self._is_auto:
            return

        y = float(msg.Y1) / 32768 * 100
        x = float(msg.X2) / 32768 * 100 
        self._publish_motor_command(x, y)

    def _publish_motor_command(self, x, y):
        data = {
            "x": x,
            "y": y
        }

        data_json = json.dumps(data)
        self._pub.publish(String(data_json))

    def _start_collision_avoidance(self):
        if self._is_avoiding_collision:
            return

        self._is_avoiding_collision = True 
        rospy.logerr("Timer set")
        rospy.Timer(rospy.Duration(3), self._stop_collision_avoidance, True)

    def _stop_collision_avoidance(self, event):
        self._is_avoiding_collision = False

    def run_auto(self):
        if self._is_avoiding_collision:
            rospy.loginfo("back up to the right")
            self._publish_motor_command(100, -25)
        else:
            rospy.loginfo("Move forward")
            self._publish_motor_command(0, 40)

    def run(self):
        while not rospy.is_shutdown():
            if self._is_auto:
                self.run_auto()
            
            rospy.sleep(RATE)           

if __name__ == '__main__':
    rover = Controller(DISTANCE_THRESHOLD)
    rover.run()
