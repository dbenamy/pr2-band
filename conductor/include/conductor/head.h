#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

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

  private:

    ros::NodeHandle nh_;
    PointHeadClient* point_head_client_;
};

