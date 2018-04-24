#!/usr/bin/python
import rospy
import tf
from nav_msgs.msg import Odometry

class Odometry:
    def __init__(self):
        rospy.init_node("rvr_odom")
        self.pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_br = tf.TransformBroadcaster()

        self.curr_time = rospy.Time.now()
        self.last_time = self.curr_time

    def run(self):
        x = 0.0
        y = 0.0
        th = 0.0

        vx = 0.1
        vy = -0.1
        vth = 0.1

        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            self.curr_time = rospy.Time.now()

            # Compute odometry in a typical way given the velocities of the robot
            dt = (self.curr_time - self.last_time).to_sec()
            delta_x = (vx * cos(th) - vy * sin(th)) * dt
            delta_y = (vx * sin(th) + vy * cos(th)) * dt
            delta_th = vth * dt

            x += delta_x
            y += delta_y
            th += delta_th

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
            msg = Quaternion(*odom_quat)



            rate.sleep()

if __name__ == "__main__":
    o = Odometry()
    o.run()
