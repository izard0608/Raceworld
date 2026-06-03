#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub2");
  ros::NodeHandle node;
  geometry_msgs::Twist msg;
  ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/test2", 10);
  while(ros::ok())
  {
    ros::spinOnce();
    msg.linear.x = 1.0;
    msg.angular.z = 1.0;
    pub.publish(msg);
  }
  return 0;
}
