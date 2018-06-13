import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class FollowBall:
    def __init__(self):
        self.sub = rospy.Subscriber("camera/image", Image, self.cb)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.im_pub = rospy.Publisher("processed/image", Twist, queue_size=10)
        self.bridge = CvBridge()
        self.im = np.zeros([410,308,3], dtype=np.uint8)
        self.greenLower = (29, 86, 6)
        self.greenUpper = (64, 255, 255)

    def cb(self, data):
        try: 
            self.im = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def process_image(self):
        """https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        """
        # blur frame and convert to HSV
        blurred = cv2.GaussianBlur(self.im, (11, 11), 0)
        hsv = cv2.cv2Color(blurred, cv2.COLOR_BGR2HSV)

        # Construct mask for color green and apply to im
        mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # Find largest contour and compute minimum closing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"]/M["m00"]))

            # only proceed if radius meets a minimum size
            if radius > 10:
                # draw circle and centroid on the frame
                cv2.circle(self.im, (int(x), int(y)), int(radius)),
                    (0, 255, 255), 2)
                cv2.circle(self.im, center, 5, (0, 0, 255), -1)

    def publish_image(self):
        try:
            self.im_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def compute_action(self):
        pass

    def publish_action(self):
        pass

    def run(self):
        r = rospy.rate(10)
        while not rospy.is_shutdown():
            rospy.spinOnce()
            self.process_image()
            self.publish_image()
            self.compute_action()
            self.publish_action()
            r.sleep()

if __name__ == '__main__':
    fb = FollowBall()
    fb.run()
