#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import tf
import math
import sys

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
    
def follow():
    global PI, pose_flag0, pose_flag1, initial_distance, thresh_distance, target_point, ugv0_msg, ugv1_msg, leader_msg_queue, msg, pub
    if not pose_flag0 or not pose_flag1: #接收到odometry消息后才开始运行
        return
    leader_msg_queue.append(ugv0_msg)
    
    #求目标点当前距离
    target_distance = math.sqrt((target_point.pose.pose.position.x - ugv1_msg.pose.pose.position.x) ** 2 + (target_point.pose.pose.position.y - ugv1_msg.pose.pose.position.y) ** 2)
    #求前后车当前距离
    follower_distance = math.sqrt((ugv0_msg.pose.pose.position.x - ugv1_msg.pose.pose.position.x) ** 2 + (ugv0_msg.pose.pose.position.y - ugv1_msg.pose.pose.position.y) ** 2)
    #print(target_distance, follower_distance, initial_distance)
    #前后车当前距离小于初始距离时停车
    if follower_distance < initial_distance:
        print("距离太近自动停车")
        msg.drive.speed = 0;
        msg.drive.steering_angle = 0;
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg);
    
    else:
        #target_point为初始值时直接赋值
        if target_point.pose.pose.position.x == 0 and target_point.pose.pose.position.y == 0:
            target_point = ugv0_msg
            #求前后车初始距离
            initial_distance = math.sqrt((ugv0_msg.pose.pose.position.x - ugv1_msg.pose.pose.position.x) ** 2 + (ugv0_msg.pose.pose.position.y - ugv1_msg.pose.pose.position.y) ** 2)
            print("两车初始距离: ",initial_distance)
        #后车接近目标点时按队列方式从数组中取出下一个目标点
        if target_distance < thresh_distance:
            target_point = leader_msg_queue[0]
            leader_msg_queue = leader_msg_queue[1:]
        #后车与目标点差距过大时清除并重开队列
        elif target_distance > 5:
            leader_msg_queue = []
            target_point = ugv0_msg
        #四元数转欧拉角
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([ugv1_msg.pose.pose.orientation.x, ugv1_msg.pose.pose.orientation.y, ugv1_msg.pose.pose.orientation.z, ugv1_msg.pose.pose.orientation.w])
        gamma = yaw
        delta = math.atan2(target_point.pose.pose.position.y-ugv1_msg.pose.pose.position.y,target_point.pose.pose.position.x-ugv1_msg.pose.pose.position.x);
        if delta < 0:
            delta += (2 * PI)
        if gamma < 0:
            gamma += (2 * PI)
        theta = delta - gamma
        if theta > PI:
            theta -= (2 * PI)
        if theta < -PI:
            theta += (2 * PI)
        r = math.sqrt((ugv0_msg.pose.pose.position.x - ugv1_msg.pose.pose.position.x) ** 2 + (ugv0_msg.pose.pose.position.y - ugv1_msg.pose.pose.position.y) ** 2)
        k = 0.5
        if r > 0.4:
            msg.drive.speed = 0.2 * r  #修改数值调节跟车时后车速度
        else:
            msg.drive.speed = 0.05
        msg.drive.steering_angle = k * theta
        #print("后车速度: ", msg.drive.speed, " 后车角度: ", msg.drive.steering_angle)
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg);
if __name__ == '__main__':
    zensha    = sys.argv[1] #前车名
    kousha    = sys.argv[2] #后车名
    frame_id  = sys.argv[3]
    node_name = sys.argv[4]

    PI = 3.1415926535
    pose_flag0 = False
    pose_flag1 = False
    initial_distance = 0 #前后车初始距离
    thresh_distance = 0.2 #后车和目标点距离差
    target_point = Odometry() #目标点
    ugv0_msg = Odometry() #前车里程计消息
    ugv1_msg = Odometry() #后车里程计消息
    leader_msg_queue = []
    #新建Ackermann消息
    msg = AckermannDriveStamped()
    msg.header.frame_id = frame_id
    msg.drive.acceleration = 1
    msg.drive.jerk = 1
    msg.drive.steering_angle_velocity = 1
    
    rospy.init_node(node_name)
    pub = rospy.Publisher("%s/ackermann_cmd_mux/output"%(kousha), AckermannDriveStamped,queue_size=100)
    # 订阅前车ugv0里程计消息
    rospy.Subscriber("/%s/base_pose_ground_truth"%(zensha), Odometry, pose_callback0,queue_size=10)
    # 订阅后车ugv1里程计消息
    rospy.Subscriber("/%s/base_pose_ground_truth"%(kousha), Odometry, pose_callback1,queue_size=10)
    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        follow()
        rate.sleep()
