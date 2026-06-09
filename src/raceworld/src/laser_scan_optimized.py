#! /usr/bin/env python3
# -*- coding:UTF-8 -*-
import imghdr
import rospy, cv2, cv_bridge
import sensor_msgs.msg
import random
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from itertools import *
from operator import itemgetter
import time

turn_angle = 90
THRESHOLD = 1.5 #THRESHOLD value for laser scan.
PI = 3.14
Kp1 = 0.047
Kp2 = 0.0325
Kp3 = 2
flag = True
def check_gap(i_x):
    i,x = i_x
    return i-x
def LaserScanProcess(data):
    range_angels = np.arange(len(data.ranges))
    ranges = np.array(data.ranges)
    range_mask = (ranges > THRESHOLD)
    ranges = list(range_angels[range_mask])
    # print(ranges)
    gap_list = []
    for k, g in groupby(enumerate(ranges), check_gap):
        #gap_list.append(map(itemgetter(1), g))
        gap_list.append(list(map(itemgetter(1), g)))
    gap_list.sort(key=len)
    largest_gap = gap_list[-1]
    min_angle, max_angle = largest_gap[0]*((data.angle_increment)*180/PI), largest_gap[-1]*((data.angle_increment)*180/PI)
    average_gap = (max_angle - min_angle) / 2
    global flag
    global turn_angle
    turn_angle = min_angle + average_gap

    if average_gap < 35:
        stop(1)
        print(average_gap)
        print("stop now!")
        time.sleep(3)
        rospy.signal_shutdown("~~~")
    elif turn_angle < 40 or turn_angle > 140:
        flag = False
        LINX = 0.028
        angz = Kp1 * (-1) * (90 - turn_angle)
        print("前方检测到障碍物")
        command = Twist()
        print(angz,turn_angle)
        print("正在避让\n")
        command.linear.x = LINX
        command.angular.z = angz
        cmd_vel_pub.publish(command)
    elif (turn_angle < 48 and turn_angle > 40) or (turn_angle > 132 and turn_angle < 140):
        flag = False
        LINX = 0.038
        angz = Kp2 * (-1) * (90 - turn_angle)
        print("侧前方检测到障碍物")
        command = Twist()
        print(angz,turn_angle)
        print("正在避让\n")
        command.linear.x = LINX
        command.angular.z = angz
        cmd_vel_pub.publish(command)
    elif (turn_angle < 50 and turn_angle > 48) or (turn_angle > 130 and turn_angle < 132):
        flag = False
        LINX = 0.087
        angz = Kp3 / (90 - turn_angle)
        print("侧面检测到障碍物")
        command = Twist()
        print(angz,turn_angle)
        print("正在避让\n")
        command.linear.x = LINX
        command.angular.z = angz
        cmd_vel_pub.publish(command)
    else:
        flag = True
    pass

def set_roi_forward(h, w, mask):
    search_top = int(3 * h / 4)
    search_bot = search_top + 20
    #感兴趣区域外全部改为黑色
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    return mask
    pass

def follow_line(image, color):
    cmd_vel_pub = rospy.Publisher("cmd_akm1", Twist, queue_size=10)

    #转换为HSV图像
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #cv2.namedWindow("hsv image",0)
    #cv2.imshow("hsv image",hsv)
    lower_yellow = np.array([26, 43, 46])
    upper_yellow = np.array([34, 255, 255])
    #转换为二值图，黄色范围内的车道线为白色，其他范围为黑色
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    #cv2.namedWindow("binary image",0)
    #cv2.imshow("binary image",mask)

    h, w = mask.shape
    #print(mask.shape)
    
    #设置感兴趣区域ROI
    mask = set_roi_forward(h, w, mask)
    #cv2.namedWindow("region of interest",0)
    #cv2.imshow("region of interest",mask)

    #获得图像矩
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])-235
        cy = int(M['m01'] / M['m00'])
        # print(cx, cy)
        #在质心画圆
        cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
        if flag:
            err = cx - w / 2 - 50
            twist = Twist()
            twist.linear.x = 0.08
            twist.angular.z = -float(err / 2.0) / 50
            cmd_vel_pub.publish(twist)
        else:
            rospy.sleep(4.278)
    cv2.namedWindow("camera",0)
    cv2.imshow("camera", image)
    cv2.waitKey(1)
    pass

def stop(id):
    index = id
    cmd_vel_pub = rospy.Publisher("cmd_akm"+str(index), Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    for i in range(20):
        cmd_vel_pub.publish(twist)
    time.sleep(2)

def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    follow_line(frame, "yellow")
    pass

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    cmd_vel_pub = rospy.Publisher('cmd_akm1', Twist, queue_size=10)
    rospy.Subscriber("scan", sensor_msgs.msg.LaserScan , LaserScanProcess)
    rospy.Subscriber("/car1/camera/zed_left/image_rect_color_left", Image, image_callback)    
    rate = rospy.Rate(10) #10hz
    rospy.spin()
    pass
