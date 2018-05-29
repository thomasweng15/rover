#!/usr/bin/python
import rospy
import tf
import math
from rover.msg import BoolStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class WheelOdom:
    def __init__(self):
        self.forward_ticks = []
        self.backward_ticks = []
        self.meters_per_tick = 0.3175

    def cb(self, msg):
        if msg.is_forward.data:
            self.forward_ticks.append(msg.header.stamp)
        else:
            self.backward_ticks.append(msg.header.stamp)

    def compute_velocity(self, duration):
        total_ticks = len(self.forward_ticks) - len(self.backward_ticks)
        distance = total_ticks * self.meters_per_tick
        meters_per_sec = distance * (1.0 / duration)
        return meters_per_sec

    def clear_queue(self):
        self.forward_ticks = []
        self.backward_ticks = []

class Odom:
    def __init__(self):
        rospy.init_node("rvr_odom")

        self.wheel_l = WheelOdom()
        self.wheel_r = WheelOdom()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.hz = 20
        self.wheel_radius = 0.03175
        self.wheel_separation = 0.14

        self.pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.sub_l = rospy.Subscriber("encoder_l", BoolStamped, self.wheel_l.cb)
        self.sub_r = rospy.Subscriber("encoder_r", BoolStamped, self.wheel_r.cb)
        self.odom_br = tf.TransformBroadcaster()

    def _compute_velocity(self, dt):
        v_l = self.wheel_l.compute_velocity(dt)
        v_r = self.wheel_r.compute_velocity(dt)
        
        vx = (self.wheel_radius / 2) * (v_l + v_r) * math.cos(self.th)
        vy = (self.wheel_radius / 2) * (v_l + v_r) * math.sin(self.th)
        vth = (self.wheel_radius / self.wheel_separation) * (v_r - v_l)

        return vx, vy, vth

    def _update_position(self, dt, vx, vy, vth):
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

    def run(self):
        self.curr_time = rospy.Time.now()
        self.last_time = self.curr_time

        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.curr_time = rospy.Time.now()
            dt = (self.curr_time - self.last_time).to_sec()

            vx, vy, vth = self._compute_velocity(dt)
            self._update_position(dt, vx, vy, vth)

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # first, publish transform over TF
            self.odom_br.sendTransform(
                (self.x, self.y, 0.),
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
            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # publish the message
            self.pub.publish(odom)

            self.last_time = self.curr_time
            self.wheel_l.clear_queue()
            self.wheel_r.clear_queue()
            rate.sleep()

if __name__ == "__main__":
    o = Odom()
    o.run()
