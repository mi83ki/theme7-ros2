#include "ros/ros.h"
//#include <cstdio>
//#include <iostream>
//#include <sstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define VEL_MAX 0.7
#define ACC1 0.2
#define ACC2 -0.2
#define ACC3 -0.7
#define ACC4 -0.2
#define ACC5 0.2
#define VEL_BACK -0.2
#define WHEEL_TRACK 0.125/2    //[m]

typedef struct{
  float v;
  float a;
  float omega;
}stateType;

using namespace std;

class JoyController
{
public:
  JoyController();
  void JoyUpdater();

private:
  void JoyCB(const sensor_msgs::Joy::ConstPtr& msg);

public:
  ros::Publisher pub_twist;
  geometry_msgs::Twist marica_vel;

private:
  ros::NodeHandle nh;
  ros::Subscriber sub_joy;
  sensor_msgs::Joy joy_msg;
  stateType state;
  bool back_flg;
};

using namespace std;

JoyController::JoyController() {
  pub_twist = nh.advertise<geometry_msgs::Twist>("arduino_cmd_vel", 10);
  sub_joy = nh.subscribe("joy", 10, &JoyController::JoyCB, this);

}

void JoyController::JoyCB(const sensor_msgs::Joy::ConstPtr& msg)
{
  joy_msg =*msg;
  if(state.v > 0.0) {
    //accell
    if(joy_msg.buttons[0] == 1) {
      state.a = ACC1;
    } else {
      state.a = ACC2;
    }
    //brake
    if(joy_msg.buttons[2] == 1) {  
      state.a = ACC3;
    }
  } else if(state.v < 0.0) {
    //back
    if(joy_msg.buttons[2] == 1) {
      state.a = ACC4;
    } else {
      state.a = ACC5;
    }
  
    //accell
    if(joy_msg.buttons[0] == 1)     {
      state.a = ACC1;
    }
  } else {
    //back
    if(joy_msg.buttons[2] == 1) {
      state.a = ACC4;
      back_flg = true;
    }
    //accell
    if(joy_msg.buttons[0] == 1) {
      state.a = ACC1;
      back_flg = false;
    }
  }

  //handle
  state.omega = - joy_msg.axes[0] * state.v / WHEEL_TRACK ;

}

void JoyController::JoyUpdater()
{
  if(state.v <= VEL_MAX  && state.v >= VEL_BACK) {
    state.v += state.a * 0.1;
  }
  
  if(state.v > VEL_MAX) {
    state.v = VEL_MAX;
    state.a = 0.0;
  }
  if(state.v < VEL_BACK) { 
    state.v = VEL_BACK + 0.001;
    state.a = 0.0;
  }

  if(back_flg == false) {
    if (state.v < 0.0) {
      state.a = 0.0;
      state.v = 0.0;
    }
  } else {
    if (state.v > 0.0) {
      state.a = 0.0;
      state.v = 0.0;
    }
  }

  marica_vel.linear.x = state.v;
  marica_vel.angular.z = state.omega;    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "miya_rosmarica2");

  JoyController joyCon;

  ros::Rate r(10);
  while(ros::ok())
  {
    joyCon.JoyUpdater(); 
    joyCon.pub_twist.publish(joyCon.marica_vel);
    ros::spinOnce();
    r.sleep();
  }
}


