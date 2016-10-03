#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int16.h>

#define TICKS_PER_REVOLUTION 300
#define REVOLUTIONS_PER_TICK .0033333333333

class JetRobot : public hardware_interface::RobotHW
{
public:
  double velocity_multiplier;
  JetRobot(ros::NodeHandle nh)
  {

    ros::NodeHandle pnh("~");

    pnh.param<double>("velocity_multiplier", velocity_multiplier, 10.0);
    ROS_INFO("velocity multiplier: %f", velocity_multiplier);

    hardware_interface::JointStateHandle state_handle_left("left_wheel_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_left);

    hardware_interface::JointStateHandle state_handle_right("right_wheel_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_right);

    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle vel_handle_left(jnt_state_interface.getHandle("left_wheel_joint"), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_left);

    hardware_interface::JointHandle vel_handle_right(jnt_state_interface.getHandle("right_wheel_joint"), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_right);

    registerInterface(&jnt_vel_interface);

    motor_left_pub = nh.advertise<std_msgs::Int16>("/arduino/motor_left_speed",10);
    motor_right_pub = nh.advertise<std_msgs::Int16>("/arduino/motor_right_speed",10);
    encoder_left_sub = nh.subscribe<std_msgs::UInt64>("/arduino/encoder_left_value", 10, &JetRobot::leftEncoderCB, this);
    encoder_right_sub = nh.subscribe<std_msgs::UInt64>("/arduino/encoder_right_value", 10, &JetRobot::rightEncoderCB, this);


    prev_left = 0;
    prev_right = 0;
    encoder_left = 0;
    encoder_right = 0;
    pos[0] = 0;
    pos[1] = 0;
  }
  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const{return ros::Duration(0.01);}
  void leftEncoderCB(const std_msgs::UInt64::ConstPtr& val) {
    encoder_left = (double)val->data;
  }
  void rightEncoderCB(const std_msgs::UInt64::ConstPtr& val) {
    encoder_right = (double)val->data;
  }
  void read() {
    left_msg.data = (int)(velocity_multiplier * cmd[0]);
    right_msg.data = (int)(velocity_multiplier * cmd[1]);
    motor_left_pub.publish(left_msg);
    motor_right_pub.publish(right_msg);
  }
  void write() {
    vel[0] = REVOLUTIONS_PER_TICK * (encoder_left - prev_left) / getPeriod().toSec();
    vel[1] = REVOLUTIONS_PER_TICK * (encoder_right - prev_right) / getPeriod().toSec();
    pos[0] += REVOLUTIONS_PER_TICK * (encoder_left - prev_left);
    pos[1] += REVOLUTIONS_PER_TICK * (encoder_right - prev_right);
    prev_left = encoder_left;
    prev_right = encoder_right;
  }

private:
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::JointStateInterface jnt_state_interface;
  double pos[2];
  double vel[2];
  double eff[2];
  double cmd[2];
  std_msgs::Int16 left_msg, right_msg;
  double encoder_left, encoder_right, prev_left, prev_right;
  ros::Publisher motor_left_pub;
  ros::Publisher motor_right_pub;
  ros::Subscriber encoder_left_sub;
  ros::Subscriber encoder_right_sub;
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "jet_driver_node");
  ros::NodeHandle nh;

  JetRobot jet(nh);
  controller_manager::ControllerManager cm(&jet, nh);

  ros::Rate rate(1.0 / jet.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    jet.read();
    jet.write();
    cm.update(jet.getTime(), jet.getPeriod());
    rate.sleep();
  }
  spinner.stop();
  return 0;
}
