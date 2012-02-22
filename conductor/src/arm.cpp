#include <conductor/arm.h>
#include <time.h>

Arm::Arm(std::string arm_name)
{
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


//  traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

  while(!traj_client_->waitForServer(ros::Duration(5.0)))
    ROS_INFO("Waiting for the joint_trajectory_action server");

  ROS_INFO("Initialized.");
}

Arm::~Arm()
{
  delete traj_client_;
}

void Arm::sendArmToConfiguration(double configuration[7], double move_time)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.header.seq = 0;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.header.frame_id = reference_frame_;

  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(7);
 
  for(unsigned int i = 0; i < 7; i++)
  {
    goal.trajectory.points[0].positions[i] = configuration[i];
    goal.trajectory.joint_names.push_back(joint_names_[i]);
  }

  goal.trajectory.points[0].velocities.resize(7);
//  for (size_t j = 0; j < 7; ++j)
//    goal.trajectory.points[0].velocities[j] = 1;

  goal.trajectory.points[0].time_from_start = ros::Duration(move_time);

  double start_time = ros::Time::now().toSec();
  traj_client_->sendGoal(goal);
  ROS_DEBUG("sending goal to controller took %f seconds (no feedback here)", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
}

void Arm::sendArmToConfigurations(std::vector<std::vector<double> > &configurations, std::vector<double> move_times)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.header.seq = 0;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.header.frame_id = reference_frame_;

  goal.trajectory.points.resize(configurations.size());
 
  for(unsigned int i = 0; i < 7; i++)
    goal.trajectory.joint_names.push_back(joint_names_[i]);
  
  for(unsigned int i = 0; i < configurations.size(); i++)
  {
    goal.trajectory.points[i].time_from_start = ros::Duration(move_times[i]);

    goal.trajectory.points[i].positions.resize(7);
    for(unsigned int j = 0; j < 7; j++)
    {
      goal.trajectory.points[i].positions[j] = configurations[i][j];
    }

    goal.trajectory.points[i].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
      goal.trajectory.points[i].velocities[j] = 0.0000001;
  }
  
  double start_time = ros::Time::now().toSec();
  traj_client_->sendGoal(goal);
  ROS_DEBUG("sending goal to controller took %f seconds.", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
}

/*
void Arm::sendArmToPose(double pose[6])
{
  conductor::ExecuteCartesianIKTrajectory::Request request;
  conductor::ExecuteCartesianIKTrajectory::Response response;

  btQuaternion quaternion;
  geometry_msgs::Quaternion quaternion_msg;
  
  ros::ServiceClient client = nh_.serviceClient<conductor::ExecuteCartesianIKTrajectory>("ik_trajectory", true);

  request.header.frame_id = reference_frame_;
  request.header.stamp = ros::Time::now();

  request.poses.resize(1);
  request.poses[0].position.x = pose[0];
  request.poses[0].position.y = pose[1];
  request.poses[0].position.z = pose[2];

  quaternion.setRPY(pose[3],pose[4],pose[5]);
  tf::quaternionTFToMsg(quaternion, quaternion_msg);

  request.poses[0].orientation = quaternion_msg;

 if(client.call(request,response))
 {
   if(response.success)
     ROS_DEBUG("successfully went to pose");
   else
     ROS_ERROR("wtf bitch. can't go to pose");
 }

}
*/

void Arm::sendArmToPose(double pose[6], double move_time)
{
  btQuaternion quaternion;
  geometry_msgs::Quaternion quaternion_msg;
  quaternion.setRPY(pose[3],pose[4],pose[5]);
  tf::quaternionTFToMsg(quaternion, quaternion_msg);
  
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = pose[0];
  pose_msg.position.y = pose[1];
  pose_msg.position.z = pose[2];
  pose_msg.orientation = quaternion_msg;

  std::vector<double> jnt_pos(7,0), solution(7,0);
  
  double start_time = ros::Time::now().toSec();
  if(computeIK(pose_msg,jnt_pos,solution))
  {
    ROS_DEBUG("computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
  }
  else
    return;
  
  double configuration[7];

  for(int i = 0; i < 7; i++)
    configuration[i] = solution[i];

  sendArmToConfiguration(configuration, move_time);
}

void Arm::sendArmToPoses(std::vector<std::vector<double> > &poses, std::vector<double> move_times)
{
  geometry_msgs::Pose pose_msg;
  btQuaternion quaternion;
  geometry_msgs::Quaternion quaternion_msg;
  std::vector<std::vector<double> > configurations(poses.size(), std::vector<double> (7,0));

  std::vector<double> jnt_pos(7,0), solution(7,0);

  for(unsigned int i =0; i < poses.size(); i++)
  {
    quaternion.setRPY(poses[i][3],poses[i][4],poses[i][5]);
    tf::quaternionTFToMsg(quaternion, quaternion_msg);

    pose_msg.position.x = poses[i][0];
    pose_msg.position.y = poses[i][1];
    pose_msg.position.z = poses[i][2];
    pose_msg.orientation = quaternion_msg;

    double start_time = ros::Time::now().toSec();
    if(computeIK(pose_msg,jnt_pos,configurations[i]))
      ROS_DEBUG("[sendArmToPoses] Computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
    else
      return;
  }

  sendArmToConfigurations(configurations, move_times);
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

/*
int main(int argc, char**argv)
{
  ros::init(argc,argv,"arm");

  Arm arm("right");
  double pose[6] = {0.7,-0.2,0.74, 0,0,0};

  arm.sendArmToPose(pose);

  sleep(15);

  return 0;
}
*/

