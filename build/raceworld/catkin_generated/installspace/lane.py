#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
import time
import cv2
import numpy as np
import math

pub = rospy.Publisher("/car1/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=1)
isLeft = None

#偏离太多，降速调整方向
def adjustOrientation():
    global isLeft
    msg = AckermannDriveStamped();
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.drive.speed = 0.15
    if isLeft:
        msg.drive.steering_angle = 0.1 * 3.14
    else:
        msg.drive.steering_angle = -0.1 * 3.14
    print("Speed:",msg.drive.speed)
    print("Steering_Angle:",msg.drive.steering_angle)
    #print("adjusting!The lane is " + str(isLeft))
    pub.publish(msg)
    print("Message From lane.py Published\n")


def camera_callback(img):
    global bridge
    bridge = CvBridge()
    src = img.data
    cv_img = bridge.imgmsg_to_cv2(img, "bgr8")
    lane(cv_img)
    cv2.waitKey(1)

def lane(img):

    global isLeft

    # 距离映射 y37cm x34cm 路宽26cm
    x_cmPerPixel = 34.0 / 640.0
    y_cmPerPixel =  37.0 / 480.0
    roadWidth = 570

    y_offset = 0.0  # cm

    # 轴间距
    I = 20.0
    # 摄像头坐标系与车中心间距
    D = 28.0
    # 计算cmdSteer的系数
    k = -1.5
    #原始图像四点坐标
    src_points = np.array([[240., 200.], [1., 479.], [400., 200.], [639., 479.]], dtype="float32")
    #变换后图像四点坐标
    dst_points = np.array([[1., 1.], [1., 479.], [639., 1.], [639., 479.]], dtype="float32")
    #返回透视变换矩阵
    M = cv2.getPerspectiveTransform(src_points, dst_points)

    aP = [0.0, 0.0]
    lastP = [0.0, 0.0]
    Timer = 0

    
    
    t = 10
    b = True

    try:
        
        #在原始图像四点坐标处绘制圆形
        img = cv2.circle(img, (int(src_points[0][0]),int(src_points[0][1])), 3, (0, 0, 255), -1)
        img = cv2.circle(img, (int(src_points[1][0]),int(src_points[1][1])), 3, (0, 0, 255), -1)
        img = cv2.circle(img, (int(src_points[2][0]),int(src_points[2][1])), 3, (0, 255, 0), -1)
        img = cv2.circle(img, (int(src_points[3][0]),int(src_points[3][1])), 3, (0, 255, 0), -1)
        cv2.namedWindow("camera",0)
        cv2.imshow('camera', img)
        #色彩提取
        color_dist = {'blue': {'Lower': np.array([30, 0, 0]), 'Upper': np.array([45, 255, 255])}}
        #进行高斯滤波
        blur = cv2.GaussianBlur(img,(1,1),0)
        #转换为HSV图像
        hsv_img = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
        #cv2.namedWindow("hsv image",0)
        #cv2.imshow('hsv image', hsv_img)
        kernel = np.ones((3, 3), dtype=np.uint8)
        #腐蚀操作
        erode_hsv = cv2.erode(hsv_img,kernel, 1)
        #cv2.namedWindow("erode image",0)
        #cv2.imshow('erode image', erode_hsv)
        #转换为二值图，范围内的颜色为白色，其他范围为黑色
        inRange_hsv = cv2.inRange(erode_hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
        #cv2.namedWindow("binary image",0)
        #cv2.imshow('binary image', inRange_hsv)
        gray_img = inRange_hsv
        #利用透视变换矩阵进行图片转换
        gray_img = cv2.warpPerspective(gray_img, M, (640, 480), cv2.INTER_LINEAR)
        #cv2.namedWindow("gray image",0)
        #cv2.imshow('gray image', gray_img)
        #二值化
        ret, origin_thr = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        #cv2.namedWindow("origin_thr image",0)
        #cv2.imshow('origin_thr image', origin_thr)
        
        binary_warped = origin_thr
        histogram_x = np.sum(binary_warped[int(binary_warped.shape[0] / 2):, :], axis=0)
        lane_base = np.argmax(histogram_x)
        
        midpoint_x = int(histogram_x.shape[0] / 2)
        midpoint_x = 320

        histogram_y = np.sum(binary_warped[0:binary_warped.shape[0], :], axis=1)
        midpoint_y = 240

        upper_half_histSum = np.sum(histogram_y[0:midpoint_y])
        lower_half_histSum = np.sum(histogram_y[midpoint_y:])
        try:
            hist_sum_y_ratio = (upper_half_histSum) / (lower_half_histSum)
        except:
            hist_sum_y_ratio = 1
            
        #7个滑动窗口
        nwindows = 7
        #设置每个滑动窗口的高度
        window_height = int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        lane_current = lane_base
        #滑动窗口宽度的一半
        margin = 100
        minpix = 25

        #判断车道线在小车左边还是右边
        if nonzerox.shape[0] != 0:
            isLeft = np.mean(nonzerox) < binary_warped.shape[1] / 2

        lane_inds = []

        img1 = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR)
        #cv2.namedWindow("bgr image1",0)
        #cv2.imshow('bgr image1', img1)
        for window in range(nwindows):
            #设置滑动窗口坐标点
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_x_low = lane_current - margin
            win_x_high = lane_current + margin
            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                    nonzerox < win_x_high)).nonzero()[0]

            lane_inds.append(good_inds)

            img1 = cv2.rectangle(img1, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 3)
            if len(good_inds) > minpix:
                lane_current = int(np.mean(nonzerox[good_inds]))
            elif window >= 3:
                break
        #cv2.namedWindow("bgr image2",0)
        #cv2.imshow('bgr image2', img1)
        lane_inds = np.concatenate(lane_inds)

        pixelX = nonzerox[lane_inds]
        pixelY = nonzeroy[lane_inds]
        #modified part
        try:
            a2, a1, a0 = np.polyfit(pixelY, pixelX, 2)
        except:
            print("no image! The lane is " + str(isLeft))
            adjustOrientation()
            return 0
        
        aveX = np.average(pixelX)

        frontDistance = np.argsort(pixelY)[int(len(pixelY) / 8)]
        aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]

        ploty = np.array(list(set(pixelY)))
        plotx = a2 * ploty ** 2 + a1 * ploty + a0
        num = 0
        for num in range(len(ploty) - 1):
            cv2.line(img1, (int(plotx[num]), int(ploty[num])), (int(plotx[num + 1]), int(ploty[num + 1])),
                        (0, 0, 255), 8)

        #cv2.namedWindow("bgr image3",0)
        #cv2.imshow('bgr image3', img1)
        # 计算aimLaneP处斜率，从而得到目标点的像素坐标
        lanePk = 2 * a2 * aimLaneP[0] + a1

        if abs(lanePk) < 0.1:
            if lane_base >= midpoint_x:
                LorR = -1
                #print('right1')
            else:
                if hist_sum_y_ratio < 0.1:
                    LorR = -1
                    #print('right2')
                else:
                    LorR = 1
                    #print('left1')
            aP[0] = aimLaneP[0] + LorR * roadWidth / 2
      
            aP[1] = aimLaneP[1]
        else:
            
            x_intertcept = a2 * 480.0 ** 2 + a1 * 480.0 + a0
            
            if x_intertcept > 320:
               
                LorR = -1
                #print("right3")
            else:
                LorR = -1  # LeftLane
                #print("left2")
            k_ver = - 1 / lanePk
            
            theta = math.atan(k_ver)
            aP[0] = aimLaneP[0] - math.sin(theta) * (LorR) * roadWidth / 2
            aP[1] = aimLaneP[1] - math.cos(theta) * (LorR) * roadWidth / 2
            

        #车道线上绘制蓝色圆
        img1 = cv2.circle(img1, (int(aimLaneP[0]), int(aimLaneP[1])), 10, (255, 0, 0), -1)
        #质心绘制绿色圆
        img1 = cv2.circle(img1, (int(aP[0]), int(aP[1])), 10, (0, 255, 0), -1)
        cv2.namedWindow("bgr image4",0)
        cv2.imshow('bgr image4', img1)
        aP[0] = (aP[0] - 320.0) * x_cmPerPixel
        aP[1] = (480.0 - aP[1]) * y_cmPerPixel + y_offset

        # 计算目标点的真实坐标
        if lastP[0] > 0.001 and lastP[1] > 0.001:
            if (((aP[0] - lastP[0]) ** 2 + (
                    aP[1] - lastP[1]) ** 2 > 2500) and Timer < 2):  # To avoid the mislead by walkers
                aP = lastP[:]
                Timer += 1
            else:
                Timer = 0

        lastP = aP[:]
        steerAngle = k * math.atan(2 * I * aP[0] / (aP[0] * aP[0] + (aP[1] + D) * (aP[1] + D)))
        

        #print("steerAngle=", steerAngle)
        st = steerAngle * 4.0 / 3.1415
        if st > 1:
            st = 1
        if st < -1:
            st = -1
        
        msg = AckermannDriveStamped();
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link1"

        msg.drive.speed = 0.1
        msg.drive.steering_angle = steerAngle * 0.65;
        print("Speed:",msg.drive.speed)
        print("Steering_Angle:",msg.drive.steering_angle)
        pub.publish(msg)
        print("Message From lan.py Published\n")
        #print("Is lane left of the car?" + str(isLeft))
    except KeyboardInterrupt:
        pass
   
    
    return 0
               
    
if __name__ == '__main__':
    try:
        rospy.init_node('lane', anonymous=True)
        rospy.Subscriber("/car1/camera/zed_left/image_rect_color_left", Image, camera_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
