#! /usr/bin/env python3
import os
import cv2
import numpy as np
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
import time
import pyapriltags
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from math import *

path = 1
id = 20
pre_id = 20


#换道
def change_path():
    global path
    if path == 0:
        path = 1
    else:
        path = 0
#读取二维码
def get_tag(frame):
    global id
    detector = pyapriltags.Detector()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)
    if tags == []:
        return False
    else:
        for tag in tags:
            print("Tag detected with id: ", tag.tag_id)
            id = tag.tag_id
        return True


def set_roi_forward1(h, w, mask):
    search_top = int(1 * h / 2)
    search_bot = search_top + 180
    #感兴趣区域外全部改为黑色
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    return mask


def set_roi_forward2(h, w, mask):
    mask[0:200, 0:w] = 0
    mask[415:h, 0:w] = 0
    mask[200:415, 100:w] = 0
    return mask


def set_roi_forward3(h, w, mask):
    mask[0:200, 0:w] = 0
    mask[200:360, 0:540] = 0
    mask[360:480, 0:w] = 0
    return mask

    
def follow_line_in(image):
    global pub
    print("内道")
    kernel = np.ones((7,7), np.uint8)
    #转换为HSV图像
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([26, 43, 46])
    lower_yellow = np.array([26, 43, 0])
    upper_yellow = np.array([34, 255, 255])
    #对车道线图像二值化
    mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    h, w = mask1.shape
    #膨胀操作
    mask1 = cv2.dilate(mask1, kernel, 2)
    #设置感兴趣区域ROI
    mask1 = set_roi_forward1(h, w, mask1)

    lower_gray = np.array([0, 0, 120])
    upper_gray = np.array([0, 0, 200])
    #对车道边界二值化
    mask2 = cv2.inRange(hsv, lower_gray, upper_gray)
    #膨胀操作
    mask2 = cv2.dilate(mask2, kernel, 2)
    #设置感兴趣区域ROI
    mask2 = set_roi_forward2(h, w, mask2)
    #cv2.namedWindow("mask1",0)
    #cv2.imshow("mask1", mask1)
    #cv2.namedWindow("mask2",0)
    #cv2.imshow("mask2", mask2)
    # print(h,w)
    #获得图像矩
    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)
    
    cx1 = 0
    cx2 = 0
    cy  = 0
    if M1['m00'] > 0:
        cx1 = int(M1['m10'] / M1['m00'])
        cy = int(M1['m01'] / M1['m00'])
    if M2['m00'] > 0:
        cx2 = int(M2['m10'] / M2['m00'])
    cx = int((cx1 + cx2) / 2)
    #在质心画圆
    cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
    #cv2.circle(image, (int(w / 2 - 60), cy), 10, (255, 0, 0), -1)
    err = cx - (w / 2 - 60)
    print(err)
    #angle = min(max(-1, -float(err / 2.0) / 20), 1)
    angle = min(max(-1, -float(err / 5.0) / 50), 1)
    akm = AckermannDriveStamped()
    akm.drive.speed = 0.06
    #print("Speed:",akm.drive.speed)
    akm.drive.steering_angle = angle
    #akm.drive.steering_angle = atan((angle / 0.1) * 0.133)
    print("Steering_Angle:",akm.drive.steering_angle)
    pub.publish(akm)
        
    cv2.namedWindow("Camera",0)
    cv2.imshow("Camera", image)
    cv2.waitKey(1)


def follow_line_out(image):
    global pub
    print("外道")
    kernel = np.ones((7, 7), np.uint8)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([26, 43, 0])
    upper_yellow = np.array([34, 255, 255])
    mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    h, w = mask1.shape
    mask1 = set_roi_forward1(h, w, mask1)
    mask1 = cv2.dilate(mask1, kernel, iterations=2)
    

    lower_gray = np.array([0, 0, 120])
    upper_gray = np.array([0, 0, 200])
    mask2 = cv2.inRange(hsv, lower_gray, upper_gray)
    kernel = np.ones((9,9), np.uint8)
    mask2 = set_roi_forward3(h, w, mask2)
    mask2 = cv2.dilate(mask2, kernel, iterations=2)
    #cv2.namedWindow("mask1",0)
    #cv2.imshow("mask1", mask1)
    #cv2.namedWindow("mask2",0)
    #cv2.imshow("mask2", mask2)
    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)
    
    cx1 = 0
    cx2 = 0
    cy  = 0
    if M1['m00'] > 0:
        cx1 = int(M1['m10'] / M1['m00'])
        cy = int(M1['m01'] / M1['m00'])
    #if M2['m00'] > 0:
    #    cx2 = int(M2['m10'] / M2['m00'])
    
    #cx = int((cx1 + cx2) / 2)

    cv2.circle(image, (cx1, cy), 20, (0, 0, 255), -1)
    #cv2.circle(image, (int(w / 2 + 60), cy), 10, (255, 0, 0), -1)
    err = cx1 - w / 2 - 50
    print(err)
    #angle = min(max(-1, -float(err / 2.0) / 20), 1)
    angle = -float(err / 320.0)
    akm = AckermannDriveStamped()
    akm.drive.speed = 0.08
    #print("Speed:",akm.drive.speed)
    akm.drive.steering_angle = angle
    #akm.drive.steering_angle = atan((angle / 0.1) * 0.133)
    print("Steering_Angle:",akm.drive.steering_angle)
    pub.publish(akm)
    
    cv2.namedWindow("Camera",0)
    cv2.imshow("Camera", image)
    cv2.waitKey(1)

def follow_line(frame):
    global path
    if(path == 0):
        follow_line_in(frame)
    else:
        follow_line_out(frame)


def stop():
    global pub
    akm = AckermannDriveStamped()
    akm.drive.speed = 0
    akm.drive.steering_angle = 0
    for i in range(20):
        pub.publish(akm)
    time.sleep(2)

def image_callback(msg):
    # 获取图像并开始寻线
    global id, pre_id, signal, start_point
    bridge = cv_bridge.CvBridge()
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    if get_tag(img):
        if id != pre_id:
            pre_id = id
            change_path()
        else:
            pass

    
    
    if signal != 1 and start_point:
        stop()
        print("Stop Now")
    else:
        follow_line(img)

def traffic_light_callback(msg):
    global signal
    signal = int(msg.data)

def pose_callback(msg):
    global init_pos, start_point
    if init_pos is None:
        init_pos = msg.pose.pose.position
        start_point = True
    else:
        distance = ((msg.pose.pose.position.x - init_pos.x) ** 2 + (msg.pose.pose.position.y - init_pos.y) ** 2 + (msg.pose.pose.position.z - init_pos.z) ** 2) ** 0.5
        if distance < 0.6:
            start_point = True
        else:
            start_point = False
            
if __name__ == '__main__':
    signal = -1 #交通灯信号
    start_point = False #是否位于初始位置附近
    init_pos = None #初始位置
    
    rospy.init_node("single_car")
    pub = rospy.Publisher("/car1/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)
    rospy.Subscriber("/car1/camera/zed_left/image_rect_color_left", Image, image_callback)
    rospy.Subscriber("/car1/base_pose_ground_truth", Odometry, pose_callback)
    rospy.Subscriber("traffic_light", String, traffic_light_callback)
    
    rospy.spin()
    pass
