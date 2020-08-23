//#include "odm_cal.h"

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "nav_msgs/Odometry.h"
#include "bits/stdc++.h"

#define WHEEL_R         (0.090)       /* 車輪半径(m) */
#define WHEEL_TREAD     (0.115)       /* 車輪間距離(m) */
#define RATE_REDC       (298.0)       /* 減速比 */
#define RATE_ENC        (3.0)         /* モータ1回転に必要なエンコーダカウント数 */


/* debug Mode */
#define DEBUG
#define DEBUG_SUBSC

typedef struct{
    struct{
        struct{
            struct{
                float_t x;
                float_t y;
                float_t z;
            }position;

            float_t th;
        }pose;
    }pose;

    struct{
        struct{
            struct{
                float_t vx;
                float_t vy;
                float_t vz;
            }linear;

            struct{
                float_t omega_x;
                float_t omega_y;
                float_t omega_z;
            }angular;
        }twist;
    }twist;
}odmType;

typedef struct{
    int32_t L;
    int32_t R;
}encType;

class OdmPublisher
{
public:
    OdmPublisher():time(0),enc(),delta_enc(),agv_vel(0),odm_state()
    {}

  void CalOdm();
//  void ~CalOdm();
  void InitOdm();

protected:
  void EncL_Callback(const std_msgs::Int32Ptr& encL_msg);
  void EncR_Callback(const std_msgs::Int32Ptr& encR_msg);
  void Time_Callback(const std_msgs::UInt32Ptr& time_msg);

  ros::Time         cuurent_time;
  ros::Publisher    odm_pub;

  ros::Subscriber   encL_sub;
  ros::Subscriber   encR_sub;
  ros::Subscriber   enc_time_sub;

  //ros::Subscriber sub_gazebo;   ///<gazebo_msgs入力ハンドラdelta_enc
  //ros::Publisher pub_pose;      ///<ロボット位置Pose 発行ハンドラ

  uint32_t time;

  encType   enc;
  encType   delta_enc;
  float_t   agv_vel;
  odmType   odm_state;

  static uint32_t pre_time;

  static encType   pre_enc;
  static float_t   pre_agv_vel;
  static odmType   pre_odm_state;
};

//int32_t OdmPublisher::enc_L;
//int32_t OdmPublisher::enc_R;
//uint32_t OdmPublisher::time;


int32_t enc_L,enc_R;
uint32_t OdmPublisher::pre_time;
encType OdmPublisher::pre_enc;
float_t OdmPublisher::pre_agv_vel;
odmType OdmPublisher::pre_odm_state;

void OdmPublisher::InitOdm()
{
    pre_time = 0;
    pre_agv_vel = 0.0;
    odmType odm_state = {0.0};
    encType pre_enc = {0};
    odmType pre_odm_state = {0.0};
}

void OdmPublisher::CalOdm()
{
  ROS_INFO("[odm_cal] init");
  ros::NodeHandle n;

  encL_sub = n.subscribe("A2_encL", 50, &OdmPublisher::EncL_Callback, this);
  encR_sub = n.subscribe("A2_encR", 50, &OdmPublisher::EncR_Callback, this);
  enc_time_sub = n.subscribe("A2_time", 50, &OdmPublisher::Time_Callback, this);
  odm_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  ROS_INFO("[odm_cal] START");

}

/*
void OdmPublisher::~CalOdm()
{

}
*/

void OdmPublisher::EncL_Callback(const std_msgs::Int32Ptr& encL_msg)
{
    enc.L = encL_msg->data;
//    ROS_INFO("pub enc.L %d", enc_L);
}

void OdmPublisher::EncR_Callback(const std_msgs::Int32Ptr& encR_msg)
{
    enc.R = encR_msg->data;
//    ROS_INFO("pub enc.R %d", enc.R);
}

void OdmPublisher::Time_Callback(const std_msgs::UInt32Ptr& time_msg)
{
  //ROS_INFO("pub time %d", time);

  time = static_cast<double>(time_msg->data);

  /* エンコーダ加算値とサンプリング時間を計算 */
  if (time == pre_time) {
      return;
  }
  ROS_INFO("[odm_cal] d_encL,R %d %d", delta_enc.L, delta_enc.R);

  delta_enc.L = enc.L - pre_enc.L;
  delta_enc.R = enc.R - pre_enc.R;

  float_t delta_t = (time - pre_time) * 0.001;
  ROS_INFO("[odm_cal] dt = %f", delta_t);

  /* 初回受信時 */
  if (pre_time == 0) {
      pre_enc.L = enc.L;
      pre_enc.R = enc.R;
      pre_time = time;

      return;
  }

  pre_enc.L = enc.L;
  pre_enc.R = enc.R;
  pre_time = time;

  /* 車輪の速度を計算 */
  float_t wheel_v_L = (float_t) (2 *M_PI *WHEEL_R *delta_enc.L) / (RATE_ENC *RATE_REDC *delta_t);
  float_t wheel_v_R = (float_t) (2 *M_PI *WHEEL_R *delta_enc.R) / (RATE_ENC *RATE_REDC *delta_t);

  /* 車体の速度と回転角速度を計算 */
  agv_vel = (wheel_v_L + wheel_v_R)/2;

  int32_t *agv_posi_ptr;
  int32_t *agv_twist_ptr;

  //agv_twist_ptr = &odm_state.twist.twist.angular;
  odm_state.twist.twist.angular.omega_z = (wheel_v_R - wheel_v_L)/(WHEEL_TREAD); //WHEEL_Tトレッド
  ROS_INFO("[odm_cal] agv_vel = %f, omega = %f", agv_vel, odm_state.twist.twist.angular.omega_z);

  /* 位置座標を計算 */
  float_t delta_th = (float_t) delta_t *(odm_state.twist.twist.angular.omega_z + pre_odm_state.twist.twist.angular.omega_z)/2;
  odm_state.pose.pose.th += delta_th;
  ROS_INFO("[odm_cal] d_th =  %f, th = %f", delta_th, odm_state.pose.pose.th);

  odm_state.twist.twist.linear.vx = agv_vel *cos(odm_state.pose.pose.th);
  odm_state.twist.twist.linear.vy = agv_vel *sin(odm_state.pose.pose.th);
  ROS_INFO("[odm_cal] vx = %f, vy = %f", odm_state.twist.twist.linear.vx, odm_state.twist.twist.linear.vy);

  float_t delta_x = (float_t) delta_t *(odm_state.twist.twist.linear.vx + pre_odm_state.twist.twist.linear.vx)/2;
  float_t delta_y = (float_t) delta_t *(odm_state.twist.twist.linear.vy + pre_odm_state.twist.twist.linear.vy)/2;
  ROS_INFO("[odm_cal] (dx, dy) = (%f, %f)",delta_x, delta_y);

  odm_state.pose.pose.position.x += delta_x;
  odm_state.pose.pose.position.y += delta_y;
  ROS_INFO("[odm_cal] (x, y) = (%f, %f)",odm_state.pose.pose.position.x, odm_state.pose.pose.position.y);

  nav_msgs::Odometry odm;

//  odm.header.stamp = cuurent_time;
//  odm.header.frame_id = "odm";
  odm.pose.pose.position.x = odm_state.pose.pose.position.x;
  odm.pose.pose.position.y = odm_state.pose.pose.position.y;
  odm.pose.pose.position.z = 0.0;

  //  θをpublishに追加
  odm.pose.pose.orientation.x = 0.0;
  odm.pose.pose.orientation.y = 0.0;
  odm.pose.pose.orientation.z = odm_state.pose.pose.th;
  odm.pose.pose.orientation.w = 0.0;

  odm.twist.twist.linear.x = odm_state.twist.twist.linear.vx;
  odm.twist.twist.linear.y = odm_state.twist.twist.linear.vy;
  odm.twist.twist.linear.z = 0.0;

  odm.twist.twist.angular.x = 0.0;
  odm.twist.twist.angular.y = 0.0;
  odm.twist.twist.angular.z = odm_state.twist.twist.angular.omega_z;

  odm_pub.publish(odm);

  pre_odm_state = odm_state;
  ROS_INFO("[odm_cal] =========================== ");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odm_cal");
  OdmPublisher OdmPub;

  OdmPub.InitOdm();
  OdmPub.CalOdm();

  ros::spin();

}
