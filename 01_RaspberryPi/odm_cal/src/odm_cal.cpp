#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <termios.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;

#define WHEEL_R         (0.090)       /* 車輪半径(m) */
#define WHEEL_TREAD     (0.120)       /* 車輪間距離(m) */
#define RATE_REDC       (150.0)       /* 減速比 */
#define RATE_ENC        (3.0)         /* モータ1回転に必要なエンコーダカウント数 */

using namespace std::chrono_literals;
class OdmPublisher : public rclcpp::Node
{
  public:
    typedef struct {
      int32_t t;
      int32_t R;
      int32_t L;
    } encType;

    OdmPublisher()
    : Node("odm_cal")
    {
      printf("OdmPublisher started!\r\n");
      pub_odm = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      printf("create publisher: odm_cal\n");
      sub_enc = this->create_subscription<std_msgs::msg::Int64MultiArray>(
        "arduino/encoders", 10, std::bind(&OdmPublisher::encCB, this, _1));
      printf("create subscriber: arduino/encoders\n");

      // タイマー周期関数作成
      timer_ = this->create_wall_timer(
        1000ms, std::bind(&OdmPublisher::timer_callback, this));

      // 変数初期化
      last_theta = 0;
      last_odm.pose.pose.position.x = 0.0;
      last_odm.pose.pose.position.y = 0.0;
      last_odm.pose.pose.position.z = 0.0;
      last_odm.twist.twist.linear.x = 0.0;
      last_odm.twist.twist.linear.y = 0.0;
      last_odm.twist.twist.linear.z = 0.0;
      last_odm.twist.twist.angular.x = 0.0;
      last_odm.twist.twist.angular.y = 0.0;
      last_odm.twist.twist.angular.z = 0.0;
    }

  private:
    // ヨー角をクオータニオンに変換する
    tf2::Quaternion createQuaternionMsgFromYaw(double yaw)
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      //return tf2::toMsg(q);
      return (q);
    }

    // オドメトリを計算する
    nav_msgs::msg::Odometry calOdm(encType delta_enc)
    {
      nav_msgs::msg::Odometry odm;

      //printf("[odm_cal] d_encR,L %d %d\n", delta_enc.R, delta_enc.L);

      float_t delta_t = (float_t)(delta_enc.t) * 0.001;
      //printf("[odm_cal] dt = %f\n", delta_t);

      /* 車輪の速度を計算 */
      float_t wheel_v_L = (M_PI * WHEEL_R * (float_t)delta_enc.L) / (RATE_ENC * RATE_REDC * delta_t);
      float_t wheel_v_R = (M_PI * WHEEL_R * (float_t)delta_enc.R) / (RATE_ENC * RATE_REDC * delta_t);

      /* 車体の速度と回転角速度を計算 */
      float_t agv_vel = (wheel_v_L + wheel_v_R) / 2;

      //agv_twist_ptr = &odm.twist.twist.angular;
      odm.twist.twist.angular.z = (wheel_v_R - wheel_v_L) / (WHEEL_TREAD); //WHEEL_Tトレッド
      //printf("[odm_cal] agv_vel = %f, omega = %f\n", agv_vel, odm.twist.twist.angular.z);

      /* 位置座標を計算 */
      float_t delta_th = (float_t) delta_t *(odm.twist.twist.angular.z + last_odm.twist.twist.angular.z) / 2;
      float_t theta = last_theta + delta_th;
      last_theta = theta;
      tf2::Quaternion q = createQuaternionMsgFromYaw(theta);
      odm.pose.pose.orientation.x = q.getX();
      odm.pose.pose.orientation.y = q.getY();
      odm.pose.pose.orientation.z = q.getZ();
      odm.pose.pose.orientation.w = q.getW();
      //printf("[odm_cal] d_th =  %f, th = %f\n", delta_th, theta);

      odm.twist.twist.linear.x = agv_vel * cos(theta);
      odm.twist.twist.linear.y = agv_vel * sin(theta);
      //printf("[odm_cal] x = %f, y = %f\n", odm.twist.twist.linear.x, odm.twist.twist.linear.y);

      float_t delta_x = (float_t) delta_t *(odm.twist.twist.linear.x + last_odm.twist.twist.linear.x) / 2;
      float_t delta_y = (float_t) delta_t *(odm.twist.twist.linear.y + last_odm.twist.twist.linear.y) / 2;
      //printf("[odm_cal] (dx, dy) = (%f, %f)\n", delta_x, delta_y);

      odm.pose.pose.position.x = last_odm.pose.pose.position.x + delta_x;
      odm.pose.pose.position.y = last_odm.pose.pose.position.y + delta_y;
      //printf("[odm_cal] (x, y) = (%f, %f)\n", odm.pose.pose.position.x, odm.pose.pose.position.y);


      //  odm.header.stamp = cuurent_time;
      //  odm.header.frame_id = "odm";
      //odm.pose.pose.position.x = odm.pose.pose.position.x;
      //odm.pose.pose.position.y = odm.pose.pose.position.y;
      odm.pose.pose.position.z = 0.0;

      // θをpublishに追加
      //odm.pose.pose.orientation = tf2::createQuaternionMsgFromYaw(theta);

      //odm.twist.twist.linear.x = odm.twist.twist.linear.x;
      //odm.twist.twist.linear.y = odm.twist.twist.linear.y;
      odm.twist.twist.linear.z = 0.0;

      //odm.twist.twist.angular.x = 0.0;
      //odm.twist.twist.angular.y = 0.0;
      //odm.twist.twist.angular.z = odm.twist.twist.angular.z;

      last_odm = odm;
      //printf("[odm_cal] =========================== \n");
      return(odm);
    }

    void encCB(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
    {
      encType delta_enc;
      delta_enc.t = msg->data[0];
      delta_enc.R = msg->data[1];
      delta_enc.L = msg->data[2];
      nav_msgs::msg::Odometry odm = calOdm(delta_enc);
      pub_odm->publish(odm);
    }

    void timer_callback()
    {
      printf("[odm_cal] (x, y, th) = (%f, %f, %f)\n", last_odm.pose.pose.position.x, last_odm.pose.pose.position.y, last_theta);
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odm;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_enc;

    float_t last_theta;
    nav_msgs::msg::Odometry last_odm;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdmPublisher>());
  rclcpp::shutdown();
  return 0;
}
