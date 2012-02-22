// @author: Joe Romano - modified from Ben Cohen's initial class

#include <pr2_impeded_joint_detector/robot_arm.h>
#include <time.h>

Arm::Arm(std::string arm_name)
{
  last_commanded_joint_states_.resize(7, 0);

  reference_frame_ = "base_link";

  arm_name_ = arm_name;

  if(arm_name_.compare("left") == 0)
  {
	ik_service_name_ = "/pr2_left_arm_kinematics/get_ik";
	joint_names_.push_back("l_shoulder_pan_joint");
	joint_names_.push_back("l_shoulder_lift_joint");
	joint_names_.push_back("l_upper_arm_roll_joint");
	joint_names_.push_back("l_elbow_flex_joint");
	joint_names_.push_back("l_forearm_roll_joint");
	joint_names_.push_back("l_wrist_flex_joint");
	joint_names_.push_back("l_wrist_roll_joint");
	traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);
  }
  else
  {
	ik_service_name_ = "/pr2_right_arm_kinematics/get_ik";
	joint_names_.push_back("r_shoulder_pan_joint");
	joint_names_.push_back("r_shoulder_lift_joint");
	joint_names_.push_back("r_upper_arm_roll_joint");
	joint_names_.push_back("r_elbow_flex_joint");
	joint_names_.push_back("r_forearm_roll_joint");
	joint_names_.push_back("r_wrist_flex_joint");
	joint_names_.push_back("r_wrist_roll_joint");
	traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
  }

  while(!traj_client_->waitForServer(ros::Duration(5.0)))
	ROS_INFO("Waiting for the joint_trajectory_action server...");

  
  ROS_INFO("Initialized.");
}

Arm::~Arm()
{
  delete traj_client_;
}

void Arm::sendArmToConfiguration(stdmat configuration, bool wait_for_result)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.header.seq = 0;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.header.frame_id = reference_frame_;

  goal.trajectory.points.resize(configuration.size());

  for(unsigned int j = 0; j < configuration.size(); j++)
  {
    goal.trajectory.points[j].positions.resize(7);
    for(unsigned int i = 0; i < 7; i++)
      goal.trajectory.points[j].positions[i] = configuration[j][i];
    goal.trajectory.points[j].velocities.resize(7);
    goal.trajectory.points[j].time_from_start = ros::Duration(configuration[j][7]);
  }
  
  for(unsigned int i = 0; i < 7; i++)
      goal.trajectory.joint_names.push_back(joint_names_[i]);

  double start_time = ros::Time::now().toSec();
  traj_client_->sendGoal(goal);

  if(wait_for_result)
    traj_client_->waitForResult();
  
  ROS_DEBUG("sending goal to controller took %f seconds (no feedback here)", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
}


void Arm::sendArmToPose(stdmat pose, bool wait_for_result)
{
  //btQuaternion quaternion;
  //geometry_msgs::Quaternion quaternion_msg;
  //quaternion.setRPY(pose[3],pose[4],pose[5]);
  //tf::quaternionTFToMsg(quaternion, quaternion_msg);
  //pose_msg.orientation = quaternion_msg;
  
  stdmat configuration;
  configuration.resize(pose.size());
  
  for(unsigned int j = 0; j < pose.size(); j++)
  {  
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = pose[j][0];
    pose_msg.position.y = pose[j][1];
    pose_msg.position.z = pose[j][2];

    pose_msg.orientation.x = pose[j][3];
    pose_msg.orientation.y = pose[j][4];
    pose_msg.orientation.z = pose[j][5];
    pose_msg.orientation.w = pose[j][6];
  
    std::vector<double> solution(7,0);
  
    double start_time = ros::Time::now().toSec();
    if(computeIK(pose_msg,last_commanded_joint_states_,solution))
    {
      ROS_DEBUG("computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
    }
    else
    {
      ROS_DEBUG("IK Failed to Computer Solution");
      return;
    }
  

    for(int i = 0; i < 7; i++)
    {
      configuration[j].resize(8);
      configuration[j][i] = solution[i];
    }
    configuration[j][7] = pose[j][7];

    // Added to get the last commanded joint_positions.
    last_commanded_joint_states_ = solution;
  }

  sendArmToConfiguration(configuration, wait_for_result);
}


bool Arm::computeIK(const geometry_msgs::Pose &pose, std::vector<double> jnt_pos, std::vector<double> &solution)
{
  kinematics_msgs::GetPositionIK::Request request;
  kinematics_msgs::GetPositionIK::Response response;
  if(arm_name_.compare("left") == 0)
    request.ik_request.ik_link_name = "l_wrist_roll_link";
  else  
    request.ik_request.ik_link_name = "r_wrist_roll_link";

  request.ik_request.pose_stamped.pose = pose;
  request.ik_request.pose_stamped.header.stamp = ros::Time();
  request.ik_request.pose_stamped.header.frame_id = reference_frame_;
  request.timeout = ros::Duration(2.0);

  request.ik_request.ik_seed_state.joint_state.header.stamp = ros::Time();
  request.ik_request.ik_seed_state.joint_state.header.frame_id = reference_frame_;
  request.ik_request.ik_seed_state.joint_state.name = joint_names_;
  request.ik_request.ik_seed_state.joint_state.position.clear();

  ROS_DEBUG("%f %f %f", pose.position.x,pose.position.y,pose.position.z);
  for(int j = 0 ; j < 7; ++j)
    request.ik_request.ik_seed_state.joint_state.position.push_back(jnt_pos[j]);

  ros::service::waitForService(ik_service_name_);
  ros::ServiceClient client = nh_.serviceClient<kinematics_msgs::GetPositionIK>(ik_service_name_, true);

  if(client.call(request, response))
  {
    ROS_DEBUG("Obtained IK solution");
    if(response.error_code.val == response.error_code.SUCCESS)
      for(unsigned int i=0; i < response.solution.joint_state.name.size(); i ++)
      {
        solution[i] = response.solution.joint_state.position[i];
        ROS_DEBUG("Joint: %s %f",response.solution.joint_state.name[i].c_str(),response.solution.joint_state.position[i]);
      }
    else
    {
      ROS_ERROR("Inverse kinematics failed for %s. (error code: %d)", request.ik_request.ik_link_name.c_str(), response.error_code.val);
      return false;
    }

    ROS_DEBUG("IK Solution");
    for(unsigned int i = 0; i < solution.size() ; ++i)
      ROS_DEBUG("%i: %f", i, solution[i]);
  }
  else
  {
    ROS_ERROR("IK service failed");
    return false;
  }
  return true;
}


