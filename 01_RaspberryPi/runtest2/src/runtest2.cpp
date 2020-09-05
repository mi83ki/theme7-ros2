#include "runtest2/runtest2.h"

// 目標座標テーブル[m]
const double OdomRun::TARGET_COORDINATES[][2] = {   
  {0.5, 0.0},
  {0.5, 0.5},
  {0.0, 0.5},
  {0.0, 0.0}
};

// コンストラクタ
OdomRun::OdomRun(ros::NodeHandle &nh):
  initial_odom(),
  initial_yaw(0.0),
  now_odom(),
  now_yaw(0.0),
  target_odom(),
  target_yaw(0.0),
  cmd_vel()
{
  sub_odom = nh.subscribe("/odom", 5, &OdomRun::callbackOdom, this);
  pub_run = nh.advertise<geometry_msgs::Twist>("arduino_cmd_vel", 5);
  num_seq = 0;
  // 目標座標をセット
  target_odom.pose.pose.position.x = TARGET_COORDINATES[num_seq][0];
  target_odom.pose.pose.position.y = TARGET_COORDINATES[num_seq][1];
  initFlag = false;
}

// デストラクタ
OdomRun::~OdomRun()
{
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;  
  pub_run.publish(cmd_vel);    
}

void OdomRun::callbackOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  now_odom = *msg;
  
  if (!initFlag) {
    ROS_INFO("[runtest2]:initFlag");
    initFlag = true;
    initial_odom = now_odom;
    initial_yaw = tf::getYaw(now_odom.pose.pose.orientation);
    ROS_INFO("[runtest2]:initial_odom.x=%f", initial_odom.pose.pose.position.x);
    ROS_INFO("[runtest2]:initial_odom.y=%f", initial_odom.pose.pose.position.y);
    ROS_INFO("[runtest2]:initial_odom.yaw=%f", initial_yaw);
  }
 
  now_odom.pose.pose.position.x -= initial_odom.pose.pose.position.x;
  now_odom.pose.pose.position.y -= initial_odom.pose.pose.position.y;
  now_yaw = tf::getYaw(now_odom.pose.pose.orientation) - initial_yaw;
  //now_yaw = cutoffAngle(now_yaw);
}

// クォータニオンから-π～πの角度を返す
double OdomRun::cutoffAngle(double yaw)
{
  int temp_n = (int)(yaw / 2 / M_PI);
  yaw -= temp_n * 2 * M_PI;

  // -2π～2πを-π～πに変換
  if (yaw < -M_PI) {
    yaw += 2 * M_PI;
  } else if (yaw > M_PI) {
    yaw -= 2 * M_PI;
  }
  return yaw;
}

// 目標地点までの距離計算
double OdomRun::calcDistOdom() 
{
  double diff_x = now_odom.pose.pose.position.x - target_odom.pose.pose.position.x;
  double diff_y = now_odom.pose.pose.position.y - target_odom.pose.pose.position.y;
  return sqrt(pow(diff_x, 2) + pow(diff_y, 2));
}

//calculate theta to Goal
double OdomRun::calcAngle(double x,double xn1,double y,double yn1)
{
  double theta;
  theta = std::atan2((yn1 - y),(xn1 - x));
  return theta;
}

// 目標座標の方向を返す[rad]
double OdomRun::getAngleError()
{
  double error = calcAngle(now_odom.pose.pose.position.x,
                    target_odom.pose.pose.position.x,
                    now_odom.pose.pose.position.y,
                    target_odom.pose.pose.position.y);
  error = cutoffAngle(now_yaw - error);
  return error;
}

// 目標地点に到着したかどうかを返す
bool OdomRun::isArrived()
{
  ROS_INFO("[runtest2]:calcDistOdom() = %.2lf, Threashold = %.2lf",
           calcDistOdom(),
           ARRIVAL_DISTANCE_TOLERANCE);
  return (calcDistOdom() < ARRIVAL_DISTANCE_TOLERANCE);
}

// まず旋回すべきかを返す
bool OdomRun::isHaveToTurn()
{
  double error = getAngleError();
  ROS_INFO("[runtest2]:getAngleError() = %.2lf, Tolerance = %.2lf",
           error, ANGLE_ERROR_TOLERANCE);
  //return (abs(error) > ANGLE_ERROR_TOLERANCE);
  if (error < 0) {
    return (error < -ANGLE_ERROR_TOLERANCE);
  } else {
    return (error > ANGLE_ERROR_TOLERANCE);
  }
}

// ゴールに辿り着いたかを返す
bool OdomRun::isGoal()
{
  return (num_seq >= sizeof(TARGET_COORDINATES)/sizeof(*TARGET_COORDINATES));
}

// 現在位置に応じて速度を設定する
void OdomRun::setVelocity()
{
  static unsigned char step = 0;
  switch(step)
  {
    case 0:
      ROS_INFO("[runtest2]:%d, (%.2lf, %.2lf, %.2lf), (%.2lf, %.2lf, %.2lf)",
               num_seq,
               target_odom.pose.pose.position.x,
               target_odom.pose.pose.position.y,
               target_yaw,
               now_odom.pose.pose.position.x,
               now_odom.pose.pose.position.y,
               now_yaw);
      if (isArrived()) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        num_seq++;
        if (!isGoal()) {
          // 目標座標をセット
          target_odom.pose.pose.position.x = TARGET_COORDINATES[num_seq][0];
          target_odom.pose.pose.position.y = TARGET_COORDINATES[num_seq][1];
        } else {
          step++;
        }
        break;
      } else {
        if (isHaveToTurn()) {
          cmd_vel.linear.x = 0.0;
          if (getAngleError() < 0) {
            cmd_vel.angular.z = VELOCITY_OMEGA;
          } else {
            cmd_vel.angular.z = -1 * VELOCITY_OMEGA;
          }
        } else {
          cmd_vel.linear.x = VELOCITY_X;
          cmd_vel.angular.z = -1 * VELOCITY_OMEGA * getAngleError();
        }
      }
      break;
    case 1: // stop
      ROS_INFO("[runtest2]:END");
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;  
      break;
    default:
      break;
  }  
  ROS_INFO("[runtest2]:cmd_vel = (%d, %d, %d), (%.2lf, %.2lf)",
           isHaveToTurn(),
           isArrived(),
           isGoal(),
           cmd_vel.linear.x,
           cmd_vel.angular.z);
}

void OdomRun::spin()
{
  //走り始める
  setVelocity();
  //pub_run.publish(cmd_vel);    
}

int main(int argc, char **argv)
{
  ROS_INFO("[runtest2]:START");
  ros::init(argc, argv, "runtest2");
  
  ros::NodeHandle nh;
  OdomRun sample_run(nh);

  ros::Rate rate(20);
  while(ros::ok())
  {
    sample_run.spin();
    ros::spinOnce();
    rate.sleep();
  }
}
