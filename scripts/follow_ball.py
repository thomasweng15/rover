import rospy

class FollowBall:
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    fb = FollowBall()
    fb.run()
