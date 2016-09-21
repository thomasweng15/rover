#!/usr/bin/python

from std_msgs.msg import String
from motors_driver import MotorsDriver
import json
import rospy

RATE = 0.2
DISTANCE_THRESHOLD = 10

class Controller:
	def __init__(self, dist_threshold):
		rospy.init_node("rvr_controller")
		self._mode = "manual" 
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

	def _mode_callback(self, msg):
		data = json.loads(msg.data)
		mode = data["mode"]		
		if mode == "auto" or mode == "manual":
			self._mode = mode
		
	def _ultrasonic_callback(self, msg):
		if self._mode == "manual":
			return

		data = json.loads(msg.data)
		distance = data["distance"]
		if distance < self._dist_threshold:
			rospy.logwarn("Detect close object: " + str(distance))
			self._start_collision_avoidance()

	def _publish_direction(self, direction, duration):
		data = { 
			"direction" : direction,
			"duration" : duration 
		}
		data_json = json.dumps(data)
		self._pub.publish(String(data_json))

	def _start_collision_avoidance(self):
		self._is_avoiding_collision = True 

	def _stop_collision_avoidance(self, event):
		self._is_avoiding_collision = False

	def run_auto(self):
		if self._is_avoiding_collision:
			rospy.loginfo("Move backward for %s", RATE * 2)
			self._publish_direction("backward", RATE * 2)
			rospy.sleep(RATE * 2)

			rospy.loginfo("Turn right for %s", RATE * 1.5)
			self._publish_direction("right", RATE * 1.5)
			
			rospy.Timer(rospy.Duration(RATE * 1.5), self._stop_collision_avoidance)
			rospy.sleep(RATE * 1.5)
		else:
			rospy.loginfo("Move forward for %s", RATE)
			self._publish_direction("forward", RATE)

			rospy.sleep(RATE)

	def run(self):
		while not rospy.is_shutdown():
			if self._mode == "auto":
				self.run_auto()
			else:
				rospy.sleep(RATE)			

if __name__ == '__main__':
	rover = Controller(DISTANCE_THRESHOLD)
	rover.run()
