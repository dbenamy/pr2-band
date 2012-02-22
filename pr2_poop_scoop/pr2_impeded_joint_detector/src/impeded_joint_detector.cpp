// @author: Joe Romano

#include <pr2_impeded_joint_detector/impeded_joint_detector.h>

ImpededJointDetector::ImpededJointDetector()  : ph_("~")
{
	if (!ph_.getParam("r1", rlim[0]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");
  if (!ph_.getParam("r2", rlim[1]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");
  if (!ph_.getParam("r3", rlim[2]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");
  if (!ph_.getParam("r4", rlim[3]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");
  if (!ph_.getParam("r5", rlim[4]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");
  if (!ph_.getParam("r6", rlim[5]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");
  if (!ph_.getParam("r7", rlim[6]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");

  if (!ph_.getParam("l1", llim[0]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");
  if (!ph_.getParam("l2", llim[1]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");
  if (!ph_.getParam("l3", llim[2]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");
  if (!ph_.getParam("l4", llim[3]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");
  if (!ph_.getParam("l5", llim[4]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");
  if (!ph_.getParam("l6", llim[5]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");
  if (!ph_.getParam("l7", llim[6]))
    ROS_ERROR("Problem getting pr2_impeded_joint_detector limit param!");

  rsub_ = nh_.subscribe("/r_arm_controller/state", 10, &ImpededJointDetector::RArmCallback, this);
  lsub_ = nh_.subscribe("/l_arm_controller/state", 10, &ImpededJointDetector::LArmCallback, this);

  rpub_ = nh_.advertise<std_msgs::Int8>("/impeded_joint_detector/right_arm", 10);
  lpub_ = nh_.advertise<std_msgs::Int8>("/impeded_joint_detector/left_arm", 10);

  ros::spin();
  
  return;
}


void ImpededJointDetector::RArmCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{

	for(int i = 0; i<7; i++)
  {
    if(fabs(msg->error.positions[i]) > rlim[i])
    {
      ROS_INFO("exceed joint r%d: (%f > %f)",i, msg->error.positions[i], rlim[i]);
      rstatus.data = i;
      rpub_.publish(rstatus);
    }
    else	rstatus.data = false;
  }
}

void ImpededJointDetector::LArmCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{

	for(int i = 0; i<7; i++)
  {
    if(fabs(msg->error.positions[i]) > llim[i])
    {
      ROS_INFO("exceed joint l%d: (%f > %f)",i, msg->error.positions[i], llim[i]);
      lstatus.data = i;
      lpub_.publish(lstatus);
    }
    else  lstatus.data = false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_impeded_joint_detector_node");

  ImpededJointDetector impeded_joint_detector;

  return 0 ;
}
