#! /usr/bin/env python3
import rospy
import sys, select, termios, tty
from ackermann_msgs.msg import AckermannDriveStamped

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
    
settings = termios.tcgetattr(sys.stdin)

def pub_cmd():
    rospy.init_node("pub_cmd")
    pub = rospy.Publisher("/car1/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)

    akm = AckermannDriveStamped()
    
    while 1:
        x=0
        a=0

        key = getKey()
        print("Key:",key.upper())
        if key == 'w':
            print("Front")
            x=0.3
            a=0
        elif key == 's':
            print("Back")
            x=-0.3
            a=0
        elif key == 'a':
            print("Front Left")
            x=0.3
            a=0.7
        elif key == 'd':
            print("Front Right")
            x=0.3
            a=-0.7
        elif key == 'x':
            print("Stop")
            x=0
            a=0
        elif key == 'o':
            print("Exit")
            break
        else:
            continue

        akm.drive.speed = x
        print("Speed:",akm.drive.speed)
        akm.drive.steering_angle = a
        print("Steering_Angle:",akm.drive.steering_angle)

        pub.publish(akm)
        print("Message From key_op.py Published\n")

if __name__=="__main__":
    pub_cmd()
    pass
