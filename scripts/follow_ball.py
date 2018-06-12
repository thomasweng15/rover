import rospy
from sensor_msgs.msg import Image

class FollowBall:
    def __init__(self):
        self.sub = rospy.Subscriber("camera/image", Image, self.cb)

    def cb(self, msg):
        pass

    def run(self):
        r = rospy.rate(10)
        while not rospy.is_shutdown():
            rospy.spinOnce()
            r.sleep()

if __name__ == '__main__':
    fb = FollowBall()
    fb.run()
