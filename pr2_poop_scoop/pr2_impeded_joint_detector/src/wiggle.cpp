// @author: Joe Romano

#include <pr2_impeded_joint_detector/props_arm_controller.h>

int main(int argc, char** argv)
{
  // initialize the ros node
  ros::init(argc, argv, "pr2_impeded_joint_wiggle_node");
  ros::NodeHandle nh;

  // instantiate some loop state variables

  // setup our arm controllers
  PropsArmController arms(nh);

  arms.wiggle();
  
  return 0;
}
