#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

#include <poop_scoop/PointHead.h>
#include <geometry_msgs/Point.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class Head
{
  public:

    // Action client initialization 
    Head();

    ~Head();

    void tilt(int pos);

    // Points the high-def camera frame at a point in a given frame  
    void lookAt(std::string frame_id, double x, double y, double z);

    // Shake the head from left to right n times  
    void shakeHead(int n);

    // Make system call to use text to speech (festival)
    void speak(std::string phrase);


  private:
    ros::NodeHandle nh_;
    PointHeadClient* point_head_client_;

    ros::ServiceServer head_point_service_;
    bool headPointService(poop_scoop::PointHead::Request &req, poop_scoop::PointHead::Response &res);

};

