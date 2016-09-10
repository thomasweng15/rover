#!/usr/bin/python

from std_msgs.msg import String
import json
import rospy

DISTANCE_THRESHOLD = 20

class Controller:
	def __init__(self, dist_threshold):
		rospy.init_node("rvr_controller")
		self._sub = rospy.Subscriber(
			"rvr_ultrasonic", 
			String, 
			self._ultrasonic_callback
		)

		self.dist_threshold = dist_threshold

	def _ultrasonic_callback(self, msg):
		data = json.loads(msg.data)
		distance = data["distance"]
		if distance < dist_threshold:
			rospy.logerr("close: " + distance)

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	rover = Controller(DISTANCE_THRESHOLD)
	rover.run()
