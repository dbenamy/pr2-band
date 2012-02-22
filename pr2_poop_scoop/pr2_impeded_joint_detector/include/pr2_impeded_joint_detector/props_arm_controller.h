// Joe Romano <joeromano@gmail.com>

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <pr2_impeded_joint_detector/robot_arm.h>

typedef std::vector<std::vector<double> >  stdmat;

using namespace std;

class PropsArmController
{
  public:
    PropsArmController(ros::NodeHandle &n);
    ~PropsArmController();  
    void wiggle();
    void resizeAndAssignTraj(double *traj,std::vector<double> *mat, double time);
    
    
    Arm* right_arm;
    Arm* left_arm;

  private:
    ros::NodeHandle nh;


};
