#! /usr/bin/env python3
import imghdr
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped

def set_roi_forward(h, w, mask):
    search_top = int(3 * h / 4)
    search_bot = search_top + 20
    #感兴趣区域外全部改为黑色
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    return mask
    pass

def follow_line(image):
    global pub

    #转换为HSV图像
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #cv2.namedWindow("hsv image",0)
    #cv2.imshow("hsv image",hsv)
    lower_yellow = numpy.array([26, 43, 46])
    upper_yellow = numpy.array([34, 255, 255])
    #转换为二值图，黄色范围内的车道线为白色，其他范围为黑色
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    #cv2.namedWindow("binary image",0)
    #cv2.imshow("binary image",mask)

    h, w = mask.shape
    #print(mask.shape)
    
    #设置感兴趣区域ROI
    mask = set_roi_forward(h, w, mask)
    cv2.namedWindow("region of interest",0)
    cv2.imshow("region of interest",mask)
    #获得图像矩
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        # print(cx, cy)
        #在质心画圆
        cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
        err = cx - w / 2 - 50
        #根据质心与图像中线的偏移设置转角
        akm = AckermannDriveStamped()
        akm.drive.speed = 0.1
        print("Speed:",akm.drive.speed)
        akm.drive.steering_angle = -float(err / 2.0) / 20
        print("Steering_Angle:",akm.drive.steering_angle)

        pub.publish(akm)
    cv2.namedWindow("camera",0)
    cv2.imshow("camera", image)
    cv2.waitKey(1)
    pass

def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    follow_line(frame)
    pass

if __name__ == '__main__':
    rospy.init_node("follower")
    pub = rospy.Publisher("/car1/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)
    rospy.Subscriber("/car1/camera/zed_left/image_rect_color_left", Image, image_callback)
    rospy.spin()
    pass
