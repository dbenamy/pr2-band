#define <iostream>
#define <trajectory_msgs/JointTrajectory.h>


class Drummer
{
  public:
    
    Drummer();

    ~Drummer(){};

    

  private:

    ros::NodeHandle nh_;
    ros::Publisher drummer_publisher_;
    ros::Subscriber song_subscriber_;
    



    void songCallback();
}


