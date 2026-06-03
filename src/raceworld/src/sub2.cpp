#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <ros/console.h>
void test2(const geometry_msgs::Twist::ConstPtr& msg);
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub2");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe<geometry_msgs::Twist>("/test2", 1, test2);
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
void test2(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("speed:%.2f,steer:%.2f", msg->linear.x, msg->angular.z);
}
