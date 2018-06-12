import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class FollowBall:
    def __init__(self):
        self.sub = rospy.Subscriber("camera/image", Image, self.cb)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.bridge = CvBridge()
        self.im = np.zeros([410,308,3], dtype=np.uint8)

    def cb(self, data):
        try: 
            self.im = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def process_image(self):
        pass

    def publish_action(self):
        pass

    def run(self):
        r = rospy.rate(10)
        while not rospy.is_shutdown():
            rospy.spinOnce()
            self.process_image()
            self.publish_action()
            r.sleep()

if __name__ == '__main__':
    fb = FollowBall()
    fb.run()
