#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

#define WHEEL_R         (0.090)       /* 車輪半径(m) */
#define WHEEL_TREAD     (0.120)       /* 車輪間距離(m) */
#define RATE_REDC       (150.0)       /* 減速比 */
#define RATE_ENC        (3.0)         /* モータ1回転に必要なエンコーダカウント数 */

using namespace std::chrono_literals;
class OdmPublisher : public rclcpp::Node
{
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odm;
    tf2_ros::TransformBroadcaster odom_transform_broadcaster;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_enc;

    float_t last_theta;
    nav_msgs::msg::Odometry last_odm;

  public:
    typedef struct {
      int32_t t;
      int32_t R;
      int32_t L;
    } encType;

    OdmPublisher()
    : Node("odm_cal"),
    odom_transform_broadcaster(this)
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
    nav_msgs::msg::Odometry pubOdm(encType delta_enc)
    {
      // 現在時刻
      rclcpp::Clock ros_clock(RCL_ROS_TIME);
      rclcpp::Time current_time = ros_clock.now();


      float_t delta_t = (float_t)(delta_enc.t) * 0.001;

      /* 車輪の速度を計算 */
      float_t wheel_v_L = (M_PI * WHEEL_R * (float_t)delta_enc.L) / (RATE_ENC * RATE_REDC * delta_t);
      float_t wheel_v_R = (M_PI * WHEEL_R * (float_t)delta_enc.R) / (RATE_ENC * RATE_REDC * delta_t);

      /* 車体の速度と回転角速度を計算 */
      float_t angular_z = (wheel_v_R - wheel_v_L) / (WHEEL_TREAD); //WHEEL_Tトレッド
      float_t delta_th = (float_t) delta_t * (angular_z + last_odm.twist.twist.angular.z) / 2;
      float_t theta = last_theta + delta_th;
      last_theta = theta;
      tf2::Quaternion q = createQuaternionMsgFromYaw(theta);
      float_t agv_vel = (wheel_v_L + wheel_v_R) / 2;
      float_t vel_x = agv_vel * cos(theta);
      float_t vel_y = agv_vel * sin(theta);

      /* 位置座標を計算 */
      float_t delta_x = (float_t) delta_t *(vel_x + last_odm.twist.twist.linear.x) / 2;
      float_t delta_y = (float_t) delta_t *(vel_y + last_odm.twist.twist.linear.y) / 2;
      float_t position_x = last_odm.pose.pose.position.x + delta_x;
      float_t position_y = last_odm.pose.pose.position.y + delta_y;

      /* odomトピックの作成 */
      nav_msgs::msg::Odometry odm;
      odm.header.stamp = current_time;
      odm.header.frame_id = "odom";
      odm.child_frame_id = "base_footprint";
      odm.pose.pose.position.x = position_x;
      odm.pose.pose.position.y = position_y;
      odm.pose.pose.position.z = 0.0;
      odm.pose.pose.orientation.x = q.getX();
      odm.pose.pose.orientation.y = q.getY();
      odm.pose.pose.orientation.z = q.getZ();
      odm.pose.pose.orientation.w = q.getW();
      odm.twist.twist.linear.x = vel_x;
      odm.twist.twist.linear.y = vel_y;
      odm.twist.twist.linear.z = 0.0;
      odm.twist.twist.angular.x = 0.0;
      odm.twist.twist.angular.y = 0.0;
      odm.twist.twist.angular.z = angular_z;
      pub_odm->publish(odm);

      geometry_msgs::msg::TransformStamped odom_transform;
      odom_transform.header.stamp = current_time;
      odom_transform.header.frame_id = "odom";
      odom_transform.child_frame_id = "base_footprint";
      odom_transform.transform.translation.x = position_x;
      odom_transform.transform.translation.y = position_y;
      odom_transform.transform.translation.z = 0.0;
      odom_transform.transform.rotation.x = q.getX();
      odom_transform.transform.rotation.y = q.getY();
      odom_transform.transform.rotation.z = q.getZ();
      odom_transform.transform.rotation.w = q.getW();
      odom_transform_broadcaster.sendTransform(odom_transform);

      last_odm = odm;
      return(odm);
    }

    void encCB(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
    {
      encType delta_enc;
      delta_enc.t = msg->data[0];
      delta_enc.R = msg->data[1];
      delta_enc.L = msg->data[2];
      pubOdm(delta_enc);
    }

    void timer_callback()
    {
      printf("[odm_cal] (x, y, th) = (%f, %f, %f)\n", last_odm.pose.pose.position.x, last_odm.pose.pose.position.y, last_theta);
    }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdmPublisher>());
  rclcpp::shutdown();
  return 0;
}
