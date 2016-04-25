#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <algorithm>
#include <sstream>

class MotorController
{
public:
  MotorController();

private:
  void twistCallback(const geometry_msgs::Twist::ConstPtr& twist);

  ros::NodeHandle nh;
  int max_speed, left_speed, right_speed;

  std_msgs::Int16 left_msg, right_msg;

  ros::Publisher motor_left_pub;
  ros::Publisher motor_right_pub;
  ros::Subscriber twist_sub;

  int last_vel;
};


MotorController::MotorController()
{
  nh.param<int>("/motor_control/max_speed", max_speed, 400);
  twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &MotorController::twistCallback, this);
  motor_left_pub = nh.advertise<std_msgs::Int16>("/arduino/motor_left_speed", 10);
  motor_right_pub = nh.advertise<std_msgs::Int16>("/arduino/motor_right_speed", 10);

  last_vel = 0;

  ros::Rate r(10);
  while(ros::ok()) {
    if(last_vel++ >= 5) {
      left_msg.data = 0;
      right_msg.data = 0;
      motor_left_pub.publish(left_msg);
      motor_right_pub.publish(right_msg);
    }
    ros::spinOnce();
    r.sleep();
  }
}

void MotorController::twistCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  last_vel = 0;

  left_speed = max_speed * (-1 * vel->linear.x + vel->angular.z);
  right_speed = max_speed *  (-1 * vel->linear.x - vel->angular.z);

  
  left_speed = std::max(std::min(left_speed, max_speed), -max_speed);
  right_speed = std::max(std::min(right_speed, max_speed), -max_speed);

  if (left_msg.data != left_speed || right_msg.data != right_speed) {
    left_msg.data = left_speed;
    right_msg.data = right_speed;
    motor_left_pub.publish(left_msg);
    motor_right_pub.publish(right_msg);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_control");

  MotorController motor_controller;

  return 0;
}

// vim: ai:ts=2:sw=2:et
