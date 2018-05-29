#!/usr/bin/python
import rospy
import tf
import math
from rover.msg import BoolStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class Odom:
    def __init__(self):
        rospy.init_node("rvr_odom")

        self.f_ticks_l = []
        self.f_ticks_r = []

        self.b_ticks_l = []
        self.b_ticks_r = []

        self.pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.sub_l = rospy.Subscriber("encoder_l", BoolStamped, self._enc_l_cb)
        self.sub_r = rospy.Subscriber("encoder_r", BoolStamped, self._enc_r_cb)
        self.odom_br = tf.TransformBroadcaster()
        self.hz = 20

    def _enc_l_cb(self, msg):
        if msg.data:
            self.f_ticks_l.append(msg.header.stamp)
        else:
            self.b_ticks_l.append(msg.header.stamp)

    def _enc_r_cb(self, msg):
        if msg.data:
            self.f_ticks_r.append(msg.header.stamp)
        else:
            self.b_ticks_r.append(msg.header.stamp)

    def _clear_queues(self):
        self.f_ticks_l = []
        self.b_ticks_l = []
        self.f_ticks_r = []
        self.b_ticks_r = []

    def run(self):
        x = 0.0
        y = 0.0
        th = 0.0

        vx = 0.1
        vy = 0.0
        vth = 0.0

        self.curr_time = rospy.Time.now()
        self.last_time = self.curr_time

        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.curr_time = rospy.Time.now()

            # TODO Compute odometry in a typical way given the velocities of the robot
            dt = (self.curr_time - self.last_time).to_sec()
            delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt;
            delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt;
            delta_th = vth * dt;

            x += delta_x;
            y += delta_y;
            th += delta_th;

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # first, publish transform over TF
            self.odom_br.sendTransform(
                (x, y, 0.),
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
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # publish the message
            self.pub.publish(odom)

            self.last_time = self.curr_time
            self._clear_queues()
            rate.sleep()

if __name__ == "__main__":
    o = Odom()
    o.run()
