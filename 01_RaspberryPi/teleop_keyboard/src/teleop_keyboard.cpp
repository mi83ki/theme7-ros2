#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <termios.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/char.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;
class KeyboardController : public rclcpp::Node
{
  public:
    KeyboardController(double linear_vel, double angular_vel, int32_t timeout)
    : Node("teleop_keyboard")
    {
      max_linear_vel = linear_vel;
      max_angular_vel = angular_vel;
      TIMEOUT_MS = timeout;
      timer_ms = 0;
      keyboard.data = '\0';
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "KeyboardController started!");
      pub_twist = this->create_publisher<geometry_msgs::msg::Twist>("arduino/cmd_vel", 10);
      RCLCPP_INFO(this->get_logger(), "create publisher: arduino/cmd_vel");
      sub_key = this->create_subscription<std_msgs::msg::Char>(
        "keyboard", 10, std::bind(&KeyboardController::keyboardCB, this, _1));
      RCLCPP_INFO(this->get_logger(), "create subscriber: joy");

      // タイマー周期関数作成
      timer_ = this->create_wall_timer(
        100ms, std::bind(&KeyboardController::timer_callback, this));
      
      RCLCPP_INFO(this->get_logger(), "control with keyboard!");
      RCLCPP_INFO(this->get_logger(), "w: forward");
      RCLCPP_INFO(this->get_logger(), "a: left");
      RCLCPP_INFO(this->get_logger(), "s: back");
      RCLCPP_INFO(this->get_logger(), "d: right");
      RCLCPP_INFO(this->get_logger(), "b or Space: stop");
    }

  private:
    void keyboardCB(const std_msgs::msg::Char::SharedPtr msg)
    {
      keyboard.data = msg->data;
      switch (keyboard.data) {
        case 'w':
          cmd_vel.linear.x = max_linear_vel;
          cmd_vel.angular.z = 0.0;
          timer_ms = 0;
          break;
        case 's':
          cmd_vel.linear.x = -max_linear_vel;
          cmd_vel.angular.z = 0.0;
          timer_ms = 0;
          break;
        case 'b':
        case ' ':
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.0;
          timer_ms = 0;
          break;
        case 'a':
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = max_angular_vel;
          timer_ms = 0;
          break;
        case 'd':
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = -max_angular_vel;
          timer_ms = 0;
          break;
        default :
          break;
      }
      pub_twist->publish(cmd_vel);
      //RCLCPP_INFO(this->get_logger(), "key: %c", keyboard.data);
    }

    void timer_callback()
    {
      if ((timer_ms >= TIMEOUT_MS) &&
          ((cmd_vel.linear.x != 0.0) || (cmd_vel.angular.z != 0.0))) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        pub_twist->publish(cmd_vel);
      }

      timer_ms += 100;
      //RCLCPP_INFO(this->get_logger(), "timer_ms: %d", timer_ms);
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist;
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr sub_key;
    std_msgs::msg::Char keyboard;
    double max_linear_vel;
    double max_angular_vel;
    int32_t TIMEOUT_MS;
    int32_t timer_ms;         // タイマカウンタ[ms]
    char key;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardController>(0.1, 0.6, 3000));
  rclcpp::shutdown();
  return 0;
}
