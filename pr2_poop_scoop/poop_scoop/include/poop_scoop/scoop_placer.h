#include <ros/ros.h>
#include <poop_scoop/arm.h>
#include <poop_scoop/ScoopPlacerSrv.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <std_msgs/Int8.h>

#include <iostream>

typedef std::vector<float> JointPos;
typedef std::vector<JointPos> Trajectory;

class ScoopPlacer
{
  public:

		ScoopPlacer();
    ~ScoopPlacer();

    int run();

    void lowerScoop();
    double lowerDistance;

  private:
    
    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    //Arm* larm_;
    Arm* rarm_;

    ros::ServiceServer scoop_placer_service_;
    bool scoopPlacerService(poop_scoop::ScoopPlacerSrv::Request &req, poop_scoop::ScoopPlacerSrv::Response &res);

    ros::Subscriber impeded_joint_sub_;
    void ImpededJointCallback(const std_msgs::Int8::ConstPtr& msg);

};

