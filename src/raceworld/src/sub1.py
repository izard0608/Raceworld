#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
def test1(msg):
    print(msg)
    pass

if __name__ == '__main__':
    rospy.init_node("sub1")
    rospy.Subscriber("/test1", Twist, test1)
    rospy.spin()
    pass
