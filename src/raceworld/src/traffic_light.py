#! /usr/bin/env python3
import rospy
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState, GetLinkState
from std_msgs.msg import String, Header
import time

#改模型红绿灯位置
#切换信号时先把之前信号模型调换复原再修改新的信号灯
def traffic_light_change(signal):
    # 根据当前信号选择需要修改的link名
    if signal == 0:
        link_name0 = 'cantilevered_light::traffic_light::left_off'
        link_name1 = 'cantilevered_light::traffic_light::left_on'
    elif signal == 1:
        link_name0 = 'cantilevered_light::traffic_light::middle_off'
        link_name1 = 'cantilevered_light::traffic_light::middle_on'
    elif signal == 2:
        link_name0 = 'cantilevered_light::traffic_light::right_off'
        link_name1 = 'cantilevered_light::traffic_light::right_on'
    
    # 获取左/中/右的红绿灯模型link state
    current_state0 = get_link_state_proxy(link_name0, reference_frame)
    link_state0 = LinkState()
    link_state0.link_name = link_name0
    current_state1 = get_link_state_proxy(link_name1, reference_frame)
    link_state1 = LinkState()
    link_state1.link_name = link_name1
    
    # 调换左/中/右的红绿灯模型
    link_state0.pose = current_state1.link_state.pose
    link_state0.twist = current_state0.link_state.twist
    link_state0.reference_frame = reference_frame
    set_link_state_proxy(link_state0)
    
    link_state1.pose = current_state0.link_state.pose
    link_state1.twist = current_state1.link_state.twist
    link_state1.reference_frame = reference_frame
    set_link_state_proxy(link_state1)

#定时切换左中右信号
def traffic_light_signal():
    global currentSignal, lastTime
    if time.time() - lastTime >= changeRate:
        traffic_light_change(currentSignal)
        currentSignal = (currentSignal + 1) % 3
        traffic_light_change(currentSignal)
        lastTime = time.time()
    pub.publish(str(currentSignal))
    rospy.loginfo("Current Traffic Light is " + str(currentSignal))

if __name__ == '__main__':
    changeRate = 8
    currentSignal = 0 #0左 1中 2右
    lastTime = time.time()

    rospy.init_node("traffic_light")
    pub = rospy.Publisher("traffic_light", String,queue_size=1)

    # 等待并调用gazebo中get_link_state和set_link_state服务
    rospy.wait_for_service('/gazebo/get_link_state')
    rospy.wait_for_service('/gazebo/set_link_state')
    get_link_state_proxy= rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    set_link_state_proxy = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
    reference_frame = 'world'
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        traffic_light_signal()
        rate.sleep()
