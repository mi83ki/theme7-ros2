#include <ros/ros.h>
#include <std_msgs/String.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define GET_ARRAY_SIZE(a)   (sizeof(a)/sizeof(a[0]))//

nav_msgs::Odometry input_msg;//subscribeしてくるpose型のメッセージを定義

float kp1 = 0; //P制御の定数
float kp2 = 0; //P制御の定数2
//const float point_array[4][2] = {{0.9,0},{0.9,0.9},{0,0.9},{0,0}};//pass point array {x,y}
const float point_array[4][2] = {{0.5,0},{0.5,0.5},{0,0.5},{0,0}};//pass point array {x,y}

float calc_angle(const float x,const float xn1,const float y,const float yn1);
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);


void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   input_msg = *msg;
   //ROS_INFO("Pose: [%f],[%f],[%f]", msg->x,msg->y,msg->theta);
}

int main(int argc, char **argv)
{
  // 初期化
    //現在位置X,Y,角度θをもらう
        ros::init(argc, argv, "runtest");
	ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("odom", 10, poseCallback); //角度，速度を読んでくるsubscriberを定義
        ros::Publisher cmd_pub= n.advertise<geometry_msgs::Twist>("arduino_cmd_vel",10);//角度,速度を配信するpublisherを定義
        ros::Rate loop_rate(20); 

        while (ros::ok())
        {
            static int point_id = 0;
            static int mode = 0; //0…角度修正モード 1…走行モード
            geometry_msgs::Twist output_msg;//publish message
            static float angular_n = 0;
            if( point_id <= GET_ARRAY_SIZE(point_array) -1 ){//https://kazuhira-r.hatenablog.com/entry/20180728/1532770315
                nav_msgs::Odometry pose_msg = input_msg;//publishするtwist型のメッセージを定義
                float xn = point_array[point_id][0];
                float yn = point_array[point_id][1];
                //float temp = pose_msg.pose.pose.orientation.z / (2.0 * M_PI);
                float temp = tf::getYaw(pose_msg.pose.pose.orientation) / (2.0 * M_PI);
                int temp_n = (int)temp;
                //temp = pose_msg.pose.pose.orientation.z - (float)temp_n * 2.0 * M_PI;
                temp = tf::getYaw(pose_msg.pose.pose.orientation) - (float)temp_n * 2.0 * M_PI;
                if( temp > M_PI){
                  temp -= 2.0 * M_PI;
                }else if( temp < - M_PI) {
                  temp += 2.0 * M_PI;
                }
                switch(mode){
                   case 0:
                    output_msg.angular.z = -1.0;
                    angular_n = calc_angle(pose_msg.pose.pose.position.x,xn,pose_msg.pose.pose.position.y,yn);
                    ROS_INFO("I want to go xn:[%f] yn:[%f]",xn,yn);
                    ROS_INFO("x:[%f] y:[%f]",pose_msg.pose.pose.position.x,pose_msg.pose.pose.position.y);
                    ROS_INFO("mode:[%d] theta:[%f] angular_n:[%f] theta_error:[%f]",
                             mode,
                             //pose_msg.pose.pose.orientation.z ,
                             temp,
                             angular_n,
                             //std::abs(pose_msg.pose.pose.orientation.z - angular_n));
                             std::abs(temp - angular_n));
                    if(std::abs(temp - angular_n) < 0.3)
                    {

                      mode = 1;
                      output_msg.angular.z = 0;

                    }
                    break;
                   case 1:
                    ROS_INFO("I want to go xn:[%f] yn:[%f]",xn,yn);
                    ROS_INFO("x:[%f] y:[%f]",pose_msg.pose.pose.position.x,pose_msg.pose.pose.position.y);
                    ROS_INFO("mode:[%d] theta:[%f] angular_n:[%f] theta_error:[%f] x_error:[%f] y_error:[%f]",
                             mode,
                             //pose_msg.pose.pose.orientation.z ,
                             temp,
                             angular_n,
                             //std::abs(pose_msg.pose.pose.orientation.z - angular_n));
                             std::abs(temp - angular_n),
                             pose_msg.pose.pose.position.x - xn,
                             pose_msg.pose.pose.position.y - yn);  
                    output_msg.linear.x = 0.10;
                    if(std::abs(temp - angular_n) > 0.3){
		       mode = 0;
                       output_msg.linear.x = 0;
                    }

                    if((std::abs(pose_msg.pose.pose.position.x -xn) < 0.1) && (std::abs(pose_msg.pose.pose.position.y -yn) < 0.1))
                    {
                       mode = 0;
                       output_msg.linear.x = 0;
                       point_id += 1;
                    }
                    break;
                   default:
                    break;
                }
                cmd_pub.publish(output_msg);
            }else{

            }

            ros::spinOnce();
            loop_rate.sleep();

        } 
  return 0;
}

//float calc_omega_p(float x,float xn,float y,float yn,float theta){
// float omega_p;
// omega_p = -kp1*(atan((yn - y)/(xn - x)) - theta);
// return omega_p;
//}
 //ωpを計算する関数

float calc_angle(const float x,const float xn1,const float y,const float yn1){ //calculate theta to Goal
    float theta;
    theta = std::atan2((yn1 - y),(xn1 - x));
    //if(theta < 0){
    //    theta = theta + 2*M_PI;
    //}
    return theta;
}
