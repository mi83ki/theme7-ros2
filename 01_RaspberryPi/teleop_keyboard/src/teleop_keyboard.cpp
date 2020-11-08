#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <termios.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Enterなしで1文字取得
int getch(void)
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char * argv[])
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("teleop_keyboard");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("arduino/cmd_vel", 10);
  auto cmd_vel = geometry_msgs::msg::Twist();
  rclcpp::WallRate rate(200ms);

  printf("control with keyboard!\r\n");
  printf("w: forward");
  printf("a: left");
  printf("s: back");
  printf("d: right");
  printf("b: stop");

  while (rclcpp::ok()) {
    char key;

    //cin >> key;
    key = getch();

    switch (key) {
      case 'w':
        cmd_vel.linear.x = 0.3;
        cmd_vel.angular.z = 0.0;
        break;
      case 's':
        cmd_vel.linear.x = -0.3;
        cmd_vel.angular.z = 0.0;
        break;
      case 'b':
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        break;
      case 'a':
        cmd_vel.linear.x = 0.3;
        cmd_vel.angular.z = -2.0;
        break;
      case 'd':
        cmd_vel.linear.x = 0.3;
        cmd_vel.angular.z = 2.0;
        break;
      default :
        break;
    }
    publisher->publish(cmd_vel);
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
