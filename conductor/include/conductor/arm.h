#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class Arm
{
  public: 
    
    Arm(std::string arm_name);
    ~Arm();

    void sendArmToPose(double pose[], double move_time);
    void sendArmToPoses(std::vector<std::vector<double> > &poses, std::vector<double> move_times);

    void sendArmToConfiguration(double configuration[], double move_time);
    void sendArmToConfigurations(std::vector<std::vector<double> > &configurations, std::vector<double> move_times);

    bool computeIK(const geometry_msgs::Pose &pose, std::vector<double> jnt_pos, std::vector<double> &solution);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_states_subscriber_;
  
    std::string arm_name_;
    std::string ik_service_name_;
    std::vector<std::string> joint_names_;
    std::string reference_frame_;

    TrajClient* traj_client_;
};


