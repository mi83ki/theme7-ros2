#include "ros/ros.h"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

using namespace std;

class JoyController
{
public:
  JoyController();
  void Spin();

private:
  void JoyCB(const sensor_msgs::Joy::ConstPtr &msg);

private:
  ros::Publisher pub_twist;
  ros::Subscriber sub_joy;
  sensor_msgs::Joy joy_msg;
  geometry_msgs::Twist marica_vel;
};

