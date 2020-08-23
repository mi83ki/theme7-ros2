#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist zero_msg;
geometry_msgs::Twist teleop_msg;
geometry_msgs::Twist processor_msg;
geometry_msgs::Twist output_msg;
std_msgs::Bool bumper_msg;
bool teleop_enable;
bool processor_enable;
bool bumper_on;

void initialize();
void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg);
void processorCallback(const geometry_msgs::Twist::ConstPtr& msg);
void bumperCallback(const std_msgs::Bool::ConstPtr& msg);
void select_msg();

/**
 * @brief メイン処理
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ailab_mux_node");
  ros::NodeHandle n;

  initialize();

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("output_cmd_vel", 100);
  ros::Subscriber teleop_vel_sub = n.subscribe("teleop_cmd_vel", 10, teleopCallback);
  ros::Subscriber processor_vel_sub = n.subscribe("processor_cmd_vel", 10, processorCallback);
  ros::Subscriber bumper_sub = n.subscribe("bumper", 10, bumperCallback);

  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    select_msg();
    cmd_vel_pub.publish(output_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

/**
 * @brief メッセージ初期化
 * 
 */
void initialize()
{
  bumper_msg.data = false;
  zero_msg.linear.x = 0.0;
  zero_msg.linear.y = 0.0;
  zero_msg.linear.z = 0.0;
  zero_msg.linear.x = 0.0;
  zero_msg.linear.y = 0.0;
  zero_msg.linear.z = 0.0;
  teleop_msg = zero_msg;
  processor_msg = zero_msg;
  output_msg = zero_msg;
}

/**
 * @brief 速度指令を選択する
 *        bumper > teleop > prosessor > zero
 */
void select_msg()
{
  if(bumper_on)
  {
    output_msg = zero_msg;
  }
  else if(teleop_enable)
  {
    output_msg = teleop_msg;
  }
  else if(processor_enable)
  {
    output_msg = processor_msg;
  }
  else
  {
    output_msg = zero_msg;
  }
  
  // フラグリセット.
  teleop_enable = false;
  processor_enable = false;
}

/**
 * @brief teleopトピックコールバック
 * 
 * @param msg 
 */
void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  teleop_msg = *msg;
  teleop_enable = true;
}
/**
 * @brief processorトピックコールバック
 * 
 * @param msg 
 */
void processorCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  processor_msg = *msg;
  processor_enable = true;
}
/**
 * @brief bumperトピックコールバック
 * 
 * @param msg 
 */
void bumperCallback(const std_msgs::Bool::ConstPtr& msg)
{
  bumper_msg = *msg;
  bumper_on = bumper_msg.data;
}
