#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

#define VELOCITY_X (0.2)      // 最大直進速度 [m/s]
#define VELOCITY_OMEGA (1.0)  // 最大旋回速度 [rad/s]
#define ARRIVAL_DISTANCE_TOLERANCE (0.05)    // 到着距離判定誤差[m]
#define ARRIVAL_ANGLE_TOLERANCE (0.05)    // 到着角度判定誤差[rad]
#define ANGLE_ERROR_TOLERANCE (0.1)    // 旋回補正を入れる許容値[rad]

class OdomRun
{
public:
  OdomRun(ros::NodeHandle &nh);
  ~OdomRun();
  void spin();
  
private:
  void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg);
  void setVelocity();
  double calcDistOdom();
  double cutoffAngle(double yaw);
  double calcAngle(double x,double xn1,double y,double yn1);
  double getAngleError();
  bool isArrived();
  bool isHaveToTurn();
  bool isGoal();

private:
  nav_msgs::Odometry initial_odom;
  double initial_yaw;
  nav_msgs::Odometry now_odom;
  double now_yaw;
  nav_msgs::Odometry target_odom;
  double target_yaw;
  geometry_msgs::Twist cmd_vel;
  ros::Subscriber sub_odom;
  ros::Publisher pub_run;
  
  bool initFlag;
  unsigned char num_seq;

  static const double TARGET_COORDINATES[][2];   // 目標座標テーブル[m]

};

