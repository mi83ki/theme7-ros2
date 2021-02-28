#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <termios.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/char.hpp"

using namespace std::chrono_literals;
class KeyboardPublisher : public rclcpp::Node
{
  public:
    KeyboardPublisher()
    : Node("pub_keyboard")
    {
      RCLCPP_INFO(this->get_logger(), "KeyboardPublisher started!");
      pub_key = this->create_publisher<std_msgs::msg::Char>("keyboard", 10);
      RCLCPP_INFO(this->get_logger(), "create publisher: keyboard");

      // タイマー周期関数作成
      timer_ = this->create_wall_timer(
        10ms, std::bind(&KeyboardPublisher::timer_callback, this));
      
      RCLCPP_INFO(this->get_logger(), "control with keyboard!");
      RCLCPP_INFO(this->get_logger(), "w: forward");
      RCLCPP_INFO(this->get_logger(), "a: left");
      RCLCPP_INFO(this->get_logger(), "s: back");
      RCLCPP_INFO(this->get_logger(), "d: right");
      RCLCPP_INFO(this->get_logger(), "b: stop");
    }

  private:
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

    void timer_callback()
    {
      std_msgs::msg::Char keyboard;
      keyboard.data = getch();
      pub_key->publish(keyboard);
      //RCLCPP_INFO(this->get_logger(), "key: %c", keyboard.data);
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr pub_key;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardPublisher>());
  rclcpp::shutdown();
  return 0;
}
