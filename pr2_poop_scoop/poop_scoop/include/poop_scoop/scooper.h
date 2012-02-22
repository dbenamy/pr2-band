#include <ros/ros.h>
#include <poop_scoop/Scooper.h>
#include <poop_scoop/arm.h>
#include <poop_scoop/head.h>
#include <poop_scoop/torso.h>
#include <poop_scoop/gripper.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <poop_scoop/visualize_arm.h>
#include <iostream>
#include <std_srvs/Empty.h>


typedef std::vector<float> JointPos;
typedef std::vector<JointPos> Trajectory;

class Scooper
{
  public:

    Scooper();
    ~Scooper();

    int run();

    void scoopPoop();

    /* sub functions of scoopPoop */
    void startScoop();
    void lowerScoop();
    void raiseScoop();
    void dropPoop();
    void returnScoop();

    bool parsePositionFile(std::string filename);

    void printDebugInfo();

    void tuckArms();

    void openScooper();

    void closeScooper();

    void performInitialScooperGrab();

  private:
    
    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Subscriber poops_sub_;
    ros::Publisher marker_publisher_;

    Arm* larm_;
    Arm* rarm_;
    Gripper* rgripper_;
    Gripper* lgripper_;
    Torso torso_;
    VisualizeArm* aviz_;
    Head head_;

    int num_poop_;

    std::vector<float> tucked_scooper_;
    std::vector<float> tucked_bucket_;
    std::vector<float> start_scooper_;
    std::vector<float> floor_scooper_;
    std::vector<float> drop_scooper_;
    std::vector<float> drop_bucket_;

    double close_scooper_position_;
    double close_scooper_effort_;
    double open_scooper_position_;
    double open_scooper_effort_;

    std::string poses_filename_;

    bool use_gripper_sensors_flag_;
    ros::ServiceClient SensorGripperService_OpenScoop_;
    ros::ServiceClient SensorGripperService_CloseScoop_;
    ros::ServiceClient SensorGripperService_OpenGripper_;
    ros::ServiceClient SensorGripperService_GrabScoop_;

    ros::ServiceServer scooping_service_;

    void placeMarker(double x, double y, double r, double g, double b);
    bool adjust_flag;
    double adjust_x;
    double adjust_y;
    bool scoopService(poop_scoop::Scooper::Request &req, poop_scoop::Scooper::Response &res);

    void poopsCallback(const sensor_msgs::PointCloudConstPtr &poop);

};

