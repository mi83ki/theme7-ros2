#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#pragma once

class OdmPublisher
{
  public:
    OdmCal();
    //~OdmCal();

  protected:
    //ros::Subscriber sub_gazebo;///<gazebo_msgs入力ハンドラ
    //ros::Publisher pub_pose;///<ロボット位置Pose 発行ハンドラ

    ros::Publisher odm_pub;
    ros::Subscriber enc_sub;
    ros::Subscriber enc_time_sub;
};
