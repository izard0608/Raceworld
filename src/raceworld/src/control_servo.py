#! /usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from math import *

def callback(msg):
    global pub

    akm = AckermannDriveStamped()

    if msg.linear.x != 0:
        akm.drive.speed = msg.linear.x*1.80
        akm.drive.steering_angle = atan((msg.angular.z/msg.linear.x)*0.133)
    else:
        akm.drive.speed = 0
        akm.drive.steering_angle = 0

    pub.publish(akm)
    pass

def cmd_to_akm():
    global pub
    rospy.init_node('control_servo', anonymous=True)
    pub = rospy.Publisher("/car1/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)
    rospy.Subscriber("cmd_akm1", Twist, callback)
    rospy.spin()
    pass

if __name__ =='__main__':
    cmd_to_akm()
    pass
