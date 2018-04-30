#!/usr/bin/python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class Odom:
    def __init__(self):
        rospy.init_node("rvr_odom")
        self.pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_br = tf.TransformBroadcaster()

        self.curr_time = rospy.Time.now()
        self.last_time = self.curr_time

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.curr_time = rospy.Time.now()

            # TODO Compute odometry in a typical way given the velocities of the robot

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)

            # first, publish transform over TF
            self.odom_br.sendTransform(
                (0., 0., 0.),
                odom_quat,
                self.curr_time,
                "base_link",
                "odom"
            )

            # next, publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = self.curr_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(0., 0., 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

            # publish the message
            self.pub.publish(odom)

            self.last_time = self.curr_time
            rate.sleep()

if __name__ == "__main__":
    o = Odom()
    o.run()
