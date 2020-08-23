#include "ros/ros.h"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <geometry_msgs/Twist.h>

using namespace std;

int main(int argc, char **argv)
{
  //init node
  ros::init(argc, argv, "rosmarica_keyboard");

  ros::NodeHandle nh;
  ros::Rate r(5);
  ros::Publisher pub;
  geometry_msgs::Twist marica_vel;

  pub = nh.advertise<geometry_msgs::Twist>("arduino_cmd_vel", 10);

  while(ros::ok())
  {
    char key;

    //cin >> key;
    key = getchar();

    switch (key) {
      case 'w':
        marica_vel.linear.x = 0.3;
        marica_vel.angular.z = 0.0;
        break;
      case 's':
        marica_vel.linear.x = -0.3;
        marica_vel.angular.z = 0.0;
        break;
      case 'b':
        marica_vel.linear.x = 0.0;
        marica_vel.angular.z = 0.0;
        break;
      case 'a':
        marica_vel.linear.x = 0.3;
        marica_vel.angular.z = -2.0;
        break;
      case 'd':
	marica_vel.linear.x = 0.3;
        marica_vel.angular.z = 2.0;
        break;
      default :
        break;
    }

    //marica_vel.linear.x = 30.0;
    //marica_vel.angular.z = 0.0;
    pub.publish(marica_vel);
    ros::spinOnce();

    r.sleep();
  }
  return 0;
}

