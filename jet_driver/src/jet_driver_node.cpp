#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class JetRobot : public hardware_interface::RobotHW
{
public:
  JetRobot() 
  { 
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
  }
  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const{return ros::Duration(0.01);}

private:
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::JointStateInterface jnt_state_interface;
  double pos[2];
  double vel[2];
  double eff[2];
  double cmd[2];
};

int main(int argc, char * argv[]) 
{
  ros::init(argc, argv, "jet_driver_node");
  ros::NodeHandle nh;

  JetRobot jet;
  controller_manager::ControllerManager cm(&jet, nh);

  ros::Rate rate(1.0 / jet.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    //jet.read();
    //jet.write();
    cm.update(jet.getTime(), jet.getPeriod());
    rate.sleep();
  }
  spinner.stop();
  return 0;
}
