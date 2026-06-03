#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
def test1():
    while True:
        pub = rospy.Publisher("/test1", Twist, queue_size=10)
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 1.0
        pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node("pub1")
    test1()
    rospy.spin()
    pass
