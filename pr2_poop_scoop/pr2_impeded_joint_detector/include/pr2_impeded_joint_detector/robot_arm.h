// Based on code written by Ben Cohen2 as part of the pr2 band.
// Modified by Will McMahan.

#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>


typedef std::vector<std::vector<double> >  stdmat;

using namespace std;

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class Arm
{
  public: 
    
    Arm(std::string arm_name);
    ~Arm();

    void sendArmToPose( stdmat pose, bool wait_for_result);
    void sendArmToConfiguration(stdmat configuration, bool wait_for_result);

    bool computeIK(const geometry_msgs::Pose &pose, std::vector<double> jnt_pos, std::vector<double> &solution);

  private:
    ros::NodeHandle nh_;
    std::vector<double> last_commanded_joint_states_;

    std::string arm_name_;
    std::string ik_service_name_;
    std::vector<std::string> joint_names_;
    std::string reference_frame_;    
   
    TrajClient* traj_client_;
 };


