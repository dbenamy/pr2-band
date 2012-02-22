#include <poop_scoop/head.h>

Head::Head()
{
  //Initialize the client for the Action interface to the head controller
  point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

  //wait for head controller action server to come up 
  while(!point_head_client_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the point_head_action server to come up");
  }

  //Advertise the head pointing service
  head_point_service_ = nh_.advertiseService("/point_head", &Head::headPointService, this);

}

Head::~Head()
{
  delete point_head_client_;
}

bool Head::headPointService(poop_scoop::PointHead::Request &req, poop_scoop::PointHead::Response &res)
{
	lookAt("base_link", req.location.x, req.location.y, req.location.z);

	res.result = 1;

	return true;
}

void Head::tilt(int pos)
{
  double x=0.3,y=0,z=0,angle=0;
  double min_z=0,max_z=2.0;
  
  angle =  double(pos)/10.0;
  z = angle*(max_z-min_z);

  lookAt("base_link",x,y,z);
}

//! Points the high-def camera frame at a point in a given frame  
void Head::lookAt(std::string frame_id, double x, double y, double z)
{
  //the goal message we will be sending
  pr2_controllers_msgs::PointHeadGoal goal;

  //the target point, expressed in the requested frame
  geometry_msgs::PointStamped point;
  point.header.frame_id = frame_id;
  point.point.x = x; point.point.y = y; point.point.z = z;
  goal.target = point;

  //we are pointing the high-def camera frame 
  //(pointing_axis defaults to X-axis)
  goal.pointing_frame = "high_def_frame";

  //take at least 0.5 seconds to get there
  goal.min_duration = ros::Duration(0.1);

  //and go no faster than 1 rad/s
  goal.max_velocity = 0.25;

  //send the goal
  point_head_client_->sendGoal(goal);


  ROS_INFO("Waiting for the head to move... if this is outside of the range, this will wait 5.0 seconds");
  //wait for it to get there
  point_head_client_->waitForResult(ros::Duration(5.0));
}

//! Shake the head from left to right n times  
void Head::shakeHead(int n)
{
  int count = 0;
  while (ros::ok() && ++count <= n )
  {
    //Looks at a point forward (x=5m), slightly left (y=1m), and 1.2m up
    lookAt("base_link", 5.0, 1.0, 1.2);

    //Looks at a point forward (x=5m), slightly right (y=-1m), and 1.2m up
    lookAt("base_link", 5.0, -1.0, 1.2);
  }
}

void Head::speak(std::string phrase)
{
  std::string command = "echo \"" + phrase + "\" | festival --tts";
  if(system(command.c_str()) == 0)
    ROS_INFO("Called festival");
  else
    ROS_WARN("festival failed.");
}

