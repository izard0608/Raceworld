#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import tf
import math

def pose_callback0(msg):
    global pose_flag0, ugv0_msg
    #print(msg)
    pose_flag0 = True
    ugv0_msg.child_frame_id = msg.child_frame_id
    ugv0_msg.header = msg.header
    ugv0_msg.pose = msg.pose
    ugv0_msg.twist = msg.twist

def pose_callback1(msg):
    global pose_flag1, ugv1_msg
    #print(msg)
    pose_flag1 = True
    ugv1_msg.child_frame_id = msg.child_frame_id
    ugv1_msg.header = msg.header
    ugv1_msg.pose = msg.pose
    ugv1_msg.twist = msg.twist

def pose_callback2(msg):
    global pose_flag2, ugv2_msg
    #print(msg)
    pose_flag2 = True
    ugv2_msg.child_frame_id = msg.child_frame_id
    ugv2_msg.header = msg.header
    ugv2_msg.pose = msg.pose
    ugv2_msg.twist = msg.twist

def pose_callback3(msg):
    global pose_flag3, ugv3_msg
    #print(msg)
    pose_flag3 = True
    ugv3_msg.child_frame_id = msg.child_frame_id
    ugv3_msg.header = msg.header
    ugv3_msg.pose = msg.pose
    ugv3_msg.twist = msg.twist
    
def follow():
    global PI, pose_flag0, pose_flag1, pose_flag2, pose_flag3, initial_distance0, initial_distance1, initial_distance2, thresh_distance, target_point0, target_point1, target_point2, ugv0_msg, ugv1_msg, ugv2_msg, ugv3_msg, leader_msg_queue0, leader_msg_queue1, leader_msg_queue2, msg0, msg1, msg2, pub0, pub1, pub2
    if not pose_flag0 or not pose_flag1 or not pose_flag2 or not pose_flag3 : #接收到odometry消息后才开始运行
        return
        
    leader_msg_queue0.append(ugv0_msg)
    
    #求目标点当前距离
    target_distance = math.sqrt((target_point0.pose.pose.position.x - ugv1_msg.pose.pose.position.x) ** 2 + (target_point0.pose.pose.position.y - ugv1_msg.pose.pose.position.y) ** 2)
    #求前后车当前距离
    follower_distance = math.sqrt((target_point0.pose.pose.position.x - ugv1_msg.pose.pose.position.x) ** 2 + (target_point0.pose.pose.position.y - ugv1_msg.pose.pose.position.y) ** 2)
    #print(target_distance, follower_distance, initial_distance)
    #前后车当前距离小于初始距离时停车
    if follower_distance < initial_distance0:
        #print("距离太近自动停车")
        msg0.drive.speed = 0;
        msg0.drive.steering_angle = 0;
        msg0.header.stamp = rospy.Time.now()
        pub0.publish(msg0);
    
    else:
        #target_point为初始值时直接赋值
        if target_point0.pose.pose.position.x == 0 and target_point0.pose.pose.position.y == 0:
            target_point0 = ugv0_msg
            #求前后车初始距离
            initial_distance0 = math.sqrt((ugv0_msg.pose.pose.position.x - ugv1_msg.pose.pose.position.x) ** 2 + (ugv0_msg.pose.pose.position.y - ugv1_msg.pose.pose.position.y) ** 2)
            print("car1 car2两车初始距离: ",initial_distance0)
        #后车接近目标点时按队列方式从数组中取出下一个目标点
        if target_distance < thresh_distance:
            target_point0 = leader_msg_queue0[0]
            leader_msg_queue0 = leader_msg_queue0[1:]
        #后车与目标点差距过大时清除并重开队列
        elif target_distance > 2:
            leader_msg_queue0 = []
            target_point0 = ugv0_msg
        #四元数转欧拉角
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([ugv1_msg.pose.pose.orientation.x, ugv1_msg.pose.pose.orientation.y, ugv1_msg.pose.pose.orientation.z, ugv1_msg.pose.pose.orientation.w])
        gamma = yaw
        delta = math.atan2(target_point0.pose.pose.position.y-ugv1_msg.pose.pose.position.y,target_point0.pose.pose.position.x-ugv1_msg.pose.pose.position.x);
        if gamma < 0:
            gamma += (2 * PI)
        theta = delta - gamma
        if theta > PI:
            theta -= (2 * PI)
        elif theta < -PI:
            theta += (2 * PI)
        r = math.sqrt((ugv0_msg.pose.pose.position.x - ugv1_msg.pose.pose.position.x) ** 2 + (ugv0_msg.pose.pose.position.y - ugv1_msg.pose.pose.position.y) ** 2)
        k = 1.0
        if r > 0.4:
            msg0.drive.speed = 0.3 * r  #修改数值调节跟车时后车速度
        else:
            msg0.drive.speed = 0.05
        msg0.drive.steering_angle = k * theta
        #print("后车car2速度: ", msg0.drive.speed, " 后车car2角度: ", msg0.drive.steering_angle)
        msg0.header.stamp = rospy.Time.now()
        pub0.publish(msg0);
        
    
    
    leader_msg_queue1.append(ugv1_msg)
    
    #求目标点当前距离
    target_distance = math.sqrt((target_point1.pose.pose.position.x - ugv2_msg.pose.pose.position.x) ** 2 + (target_point1.pose.pose.position.y - ugv2_msg.pose.pose.position.y) ** 2)
    #求前后车当前距离
    follower_distance = math.sqrt((target_point1.pose.pose.position.x - ugv2_msg.pose.pose.position.x) ** 2 + (target_point1.pose.pose.position.y - ugv2_msg.pose.pose.position.y) ** 2)
    #print(target_distance, follower_distance, initial_distance)
    #前后车当前距离小于初始距离时停车
    if follower_distance < initial_distance1:
        #print("距离太近自动停车")
        msg1.drive.speed = 0;
        msg1.drive.steering_angle = 0;
        msg1.header.stamp = rospy.Time.now()
        pub1.publish(msg1);
    
    else:
        #target_point为初始值时直接赋值
        if target_point1.pose.pose.position.x == 0 and target_point1.pose.pose.position.y == 0:
            target_point1 = ugv1_msg
            #求前后车初始距离
            initial_distance1 = math.sqrt((ugv1_msg.pose.pose.position.x - ugv2_msg.pose.pose.position.x) ** 2 + (ugv1_msg.pose.pose.position.y - ugv2_msg.pose.pose.position.y) ** 2)
            print("car2 car3两车初始距离: ",initial_distance1)
        #后车接近目标点时按队列方式从数组中取出下一个目标点
        if target_distance < thresh_distance:
            target_point1 = leader_msg_queue1[0]
            leader_msg_queue1 = leader_msg_queue1[1:]
        #后车与目标点差距过大时清除并重开队列
        elif target_distance > 2:
            leader_msg_queue1 = []
            target_point1 = ugv1_msg
        #四元数转欧拉角
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([ugv2_msg.pose.pose.orientation.x, ugv2_msg.pose.pose.orientation.y, ugv2_msg.pose.pose.orientation.z, ugv2_msg.pose.pose.orientation.w])
        gamma = yaw
        delta = math.atan2(target_point1.pose.pose.position.y-ugv2_msg.pose.pose.position.y,target_point1.pose.pose.position.x-ugv2_msg.pose.pose.position.x);
        if gamma < 0:
            gamma += (2 * PI)
        theta = delta - gamma
        if theta > PI:
            theta -= (2 * PI)
        elif theta < -PI:
            theta += (2 * PI)
        r = math.sqrt((ugv1_msg.pose.pose.position.x - ugv2_msg.pose.pose.position.x) ** 2 + (ugv1_msg.pose.pose.position.y - ugv2_msg.pose.pose.position.y) ** 2)
        k = 1.0
        if r > 0.4:
            msg1.drive.speed = 0.3 * r  #修改数值调节跟车时后车速度
        else:
            msg1.drive.speed = 0.05
        msg1.drive.steering_angle = k * theta
        #print("后车car3速度: ", msg1.drive.speed, " 后车car3角度: ", msg1.drive.steering_angle)
        msg1.header.stamp = rospy.Time.now()
        pub1.publish(msg1);
        
        
        
    leader_msg_queue2.append(ugv2_msg)
    
    #求目标点当前距离
    target_distance = math.sqrt((target_point2.pose.pose.position.x - ugv3_msg.pose.pose.position.x) ** 2 + (target_point2.pose.pose.position.y - ugv3_msg.pose.pose.position.y) ** 2)
    #求前后车当前距离
    follower_distance = math.sqrt((target_point2.pose.pose.position.x - ugv3_msg.pose.pose.position.x) ** 2 + (target_point2.pose.pose.position.y - ugv3_msg.pose.pose.position.y) ** 2)
    #print(target_distance, follower_distance, initial_distance)
    #前后车当前距离小于初始距离时停车
    if follower_distance < initial_distance2:
        #print("距离太近自动停车")
        msg2.drive.speed = 0;
        msg2.drive.steering_angle = 0;
        msg2.header.stamp = rospy.Time.now()
        pub2.publish(msg2);
    
    else:
        #target_point为初始值时直接赋值
        if target_point2.pose.pose.position.x == 0 and target_point2.pose.pose.position.y == 0:
            target_point2 = ugv2_msg
            #求前后车初始距离
            initial_distance2 = math.sqrt((ugv2_msg.pose.pose.position.x - ugv3_msg.pose.pose.position.x) ** 2 + (ugv2_msg.pose.pose.position.y - ugv3_msg.pose.pose.position.y) ** 2)
            print("car3 car4两车初始距离: ",initial_distance2)
        #后车接近目标点时按队列方式从数组中取出下一个目标点
        if target_distance < thresh_distance:
            target_point2 = leader_msg_queue2[0]
            leader_msg_queue2 = leader_msg_queue2[1:]
        #后车与目标点差距过大时清除并重开队列
        elif target_distance > 2:
            leader_msg_queue2 = []
            target_point2 = ugv2_msg
        #四元数转欧拉角
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([ugv3_msg.pose.pose.orientation.x, ugv3_msg.pose.pose.orientation.y, ugv3_msg.pose.pose.orientation.z, ugv3_msg.pose.pose.orientation.w])
        gamma = yaw
        delta = math.atan2(target_point2.pose.pose.position.y-ugv3_msg.pose.pose.position.y,target_point2.pose.pose.position.x-ugv3_msg.pose.pose.position.x);
        if gamma < 0:
            gamma += (2 * PI)
        theta = delta - gamma
        if theta > PI:
            theta -= (2 * PI)
        elif theta < -PI:
            theta += (2 * PI)
        r = math.sqrt((ugv2_msg.pose.pose.position.x - ugv3_msg.pose.pose.position.x) ** 2 + (ugv2_msg.pose.pose.position.y - ugv3_msg.pose.pose.position.y) ** 2)
        k = 1.0
        if r > 0.4:
            msg2.drive.speed = 0.3 * r  #修改数值调节跟车时后车速度
        else:
            msg2.drive.speed = 0.05
        msg2.drive.steering_angle = k * theta
        #print("后车car4速度: ", msg2.drive.speed, " 后车car4角度: ", msg2.drive.steering_angle)
        msg2.header.stamp = rospy.Time.now()
        pub2.publish(msg2);
if __name__ == '__main__':
    PI = 3.1415926535
    pose_flag0 = False
    pose_flag1 = False
    pose_flag2 = False
    pose_flag3 = False
    initial_distance0 = 0 #car1car2前后车初始距离
    initial_distance1 = 0 #car2car3前后车初始距离
    initial_distance2 = 0 #car3car4前后车初始距离
    thresh_distance = 0.2 #后车和目标点距离差
    target_point0 = Odometry() #car2目标点
    target_point1 = Odometry() #car3目标点
    target_point2 = Odometry() #car4目标点
    ugv0_msg = Odometry() #car1里程计消息
    ugv1_msg = Odometry() #car2里程计消息
    ugv2_msg = Odometry() #car3里程计消息
    ugv3_msg = Odometry() #car4里程计消息
    leader_msg_queue0 = [] #car1里程计消息队列
    leader_msg_queue1 = [] #car2里程计消息队列
    leader_msg_queue2 = [] #car3里程计消息队列
    #新建Ackermann消息
    msg0 = AckermannDriveStamped()
    msg0.header.frame_id = "base_link1"
    msg0.drive.acceleration = 1
    msg0.drive.jerk = 1
    msg0.drive.steering_angle_velocity = 1
    
    msg1 = AckermannDriveStamped()
    msg1.header.frame_id = "base_link2"
    msg1.drive.acceleration = 1
    msg1.drive.jerk = 1
    msg1.drive.steering_angle_velocity = 1
    
    msg2 = AckermannDriveStamped()
    msg2.header.frame_id = "base_link3"
    msg2.drive.acceleration = 1
    msg2.drive.jerk = 1
    msg2.drive.steering_angle_velocity = 1
    
    rospy.init_node("follow")
    pub0 = rospy.Publisher("car2/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=100)
    pub1 = rospy.Publisher("car3/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=100)
    pub2 = rospy.Publisher("car4/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=100)
    # 订阅car1里程计消息
    rospy.Subscriber("/car1/base_pose_ground_truth", Odometry, pose_callback0,queue_size=10)
    # 订阅car2里程计消息
    rospy.Subscriber("/car2/base_pose_ground_truth", Odometry, pose_callback1,queue_size=10)
    # 订阅car3里程计消息
    rospy.Subscriber("/car3/base_pose_ground_truth", Odometry, pose_callback2,queue_size=10)
    # 订阅car4里程计消息
    rospy.Subscriber("/car4/base_pose_ground_truth", Odometry, pose_callback3,queue_size=10)
    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        follow()
        rate.sleep()
