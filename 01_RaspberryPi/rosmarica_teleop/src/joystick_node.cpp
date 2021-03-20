#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <termios.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

using std::placeholders::_1;

#define VEL_MAX 0.7
#define ACC1 0.4
#define ACC2 -0.4
#define ACC3 -0.4
#define ACC4 -0.4
#define ACC5 0.4
#define VEL_BACK -0.4
#define WHEEL_TRACK 0.125/2    //[m]

typedef struct{
  float v;
  float a;
  float omega;
}stateType;

using namespace std::chrono_literals;
class JoyController : public rclcpp::Node
{
  public:
    JoyController()
    : Node("rosmarica_teleop")
    {
      RCLCPP_INFO(this->get_logger(), "JoyController started!");
      pub_twist = this->create_publisher<geometry_msgs::msg::Twist>("arduino/cmd_vel", 10);
      RCLCPP_INFO(this->get_logger(), "create publisher: arduino/cmd_vel");
      sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoyController::JoyCB, this, _1));
      RCLCPP_INFO(this->get_logger(), "create subscriber: joy");

      // タイマー周期関数作成
      timer_ = this->create_wall_timer(
        100ms, std::bind(&JoyController::timer_callback, this));
    }

  private:
    void JoyCB(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      joy_msg = *msg;
      if (state.v > 0.0) {
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
      } else if (state.v < 0.0) {
        //back
        if(joy_msg.buttons[2] == 1) {
          state.a = ACC4;
        } else {
          state.a = ACC5;
        }
        //accell
        if(joy_msg.buttons[0] == 1) {
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

    void timer_callback()
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
      pub_twist->publish(marica_vel);
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist;
    geometry_msgs::msg::Twist marica_vel;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    sensor_msgs::msg::Joy joy_msg;
    stateType state;
    bool back_flg;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyController>());
  rclcpp::shutdown();
  return 0;
}
