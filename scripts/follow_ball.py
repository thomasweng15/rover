import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class FollowBall:
    def __init__(self):
        self.sub = rospy.Subscriber("camera/image", Image, self.cb)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    def cb(self, msg):
        pass

    def publish_cmd_vel(self):
        pass

    def run(self):
        r = rospy.rate(10)
        while not rospy.is_shutdown():
            rospy.spinOnce()
            self.publish_cmd_vel()
            r.sleep()

if __name__ == '__main__':
    fb = FollowBall()
    fb.run()
