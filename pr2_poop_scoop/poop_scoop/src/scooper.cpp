#include <poop_scoop/scooper.h>

Scooper::Scooper() : ph_("~")
{
  adjust_flag = 0;
  adjust_x = 0;
  adjust_y = 0;

  ROS_INFO("Scooper is initializing.");

  scooping_service_ = nh_.advertiseService("/scoop_poop", &Scooper::scoopService,this);

  larm_ = new Arm(std::string("left"));
  rarm_ = new Arm(std::string("right"));
  aviz_ = new VisualizeArm(std::string("right_arm"));
  rgripper_ = new Gripper(std::string("right"));
  lgripper_ = new Gripper(std::string("left"));
  ph_.param<std::string>("arm_positions_filename", poses_filename_, " ");
  ph_.param<bool>("use_gripper_sensors", use_gripper_sensors_flag_, false);
  //ph_.param("close_scooper_position", close_scooper_position_, 0.86);
  //ph_.param("close_scooper_effort", close_scooper_effort_, 100.0);
  //ph_.param("open_scooper_position", open_scooper_position_, 0.0);
  //ph_.param("open_scooper_effort", open_scooper_effort_, 100.0);

  if (use_gripper_sensors_flag_)
  {
    ROS_INFO("Using the Pressure Sensors to control the Gripper!");
    SensorGripperService_OpenScoop_   = nh_.serviceClient<std_srvs::Empty>("open_scoop");
    SensorGripperService_CloseScoop_  = nh_.serviceClient<std_srvs::Empty>("close_scoop");
    SensorGripperService_OpenGripper_ = nh_.serviceClient<std_srvs::Empty>("open_gripper");
    SensorGripperService_GrabScoop_   = nh_.serviceClient<std_srvs::Empty>("grab_scoop");

    if(!SensorGripperService_OpenScoop_.waitForExistence(ros::Duration(5.0)))
      ROS_ERROR("Sensor Gripper Open Scoop Service does not exist!");
    if(!SensorGripperService_CloseScoop_.waitForExistence(ros::Duration(5.0)))
      ROS_ERROR("Sensor Gripper Close Scoop Service does not exist!");
    if(!SensorGripperService_OpenGripper_.waitForExistence(ros::Duration(5.0)))
      ROS_ERROR("Sensor Gripper Open Gripper Service does not exist!");
    if(!SensorGripperService_GrabScoop_.waitForExistence(ros::Duration(5.0)))
      ROS_ERROR("Sensor Gripper Grab Scoop Service does not exist!");
  }
  else
    ROS_INFO("Using Dumb Gripper Control!");

  lgripper_->close(0);

  sleep(1);

  if(!parsePositionFile(poses_filename_))
    ROS_ERROR("Failed to parse positions file");
  else
    ROS_INFO("Parsed the arm positions file.");
  printDebugInfo();
  ROS_INFO("Raising torso to the proper height.");
  torso_.up();
  //ROS_INFO("Tucking Arms.");
  //tuckArms();
  ROS_INFO("Scooper is initialized. Spinning.");

  poops_sub_ = nh_.subscribe("poo_view", 1, &Scooper::poopsCallback,this);
  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker",2);
}

Scooper::~Scooper()
{
  delete larm_;
  delete rarm_;
  delete lgripper_;
  delete rgripper_;
  delete aviz_;

  SensorGripperService_CloseScoop_.shutdown();
  SensorGripperService_CloseScoop_.shutdown();
  SensorGripperService_OpenGripper_.shutdown();
  SensorGripperService_GrabScoop_.shutdown();
}

void Scooper::tuckArms()
{
  closeScooper();
  larm_->sendArmToConfiguration(tucked_bucket_, 3.0);
  //sleep(3.0);
  rarm_->sendArmToConfiguration(tucked_scooper_, 3.0);
  sleep(3.0);
}

void Scooper::poopsCallback(const sensor_msgs::PointCloudConstPtr &poop)
{
  visualization_msgs::Marker marker;

  marker.header.seq = 0;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "/map";

  marker.ns = "poops";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.04;
  marker.color.r = 1.0;//120.0/255.0;
  marker.color.g = 0.0;//42.0/255.0;
  marker.color.b = 0.0;//42.0/255.0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(180.0);

  int fakes = 0;
  geometry_msgs::Point point;
  for(unsigned int i = 0; i < poop->points.size(); ++i)
  {
    if(poop->points[i].x == 25 && poop->points[i].y == 25 && poop->points[i].z == 25)
    {
      fakes++;
      continue;
    }

    //ROS_INFO("    x: %0.3f  y: %0.3f  z: %0.3f", poop->points[i].x, poop->points[i].y, poop->points[i].z);
    point.x = poop->points[i].x;
    point.y = poop->points[i].y;
    point.z = poop->points[i].z;
    marker.points.push_back(point);
  }

  //ROS_INFO("Publishing %d poop markers. Received %d fake poops. (frame: %s).", int(marker.points.size()), fakes, marker.header.frame_id.c_str());
  marker_publisher_.publish(marker);
}

void Scooper::placeMarker(double x, double y, double r, double g, double b)
{
  visualization_msgs::Marker marker;

  marker.header.seq = 0;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "/base_footprint";

  marker.ns = "arm";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.005;
  marker.scale.y = 0.04;
  marker.scale.z = 0.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration(180.0);

  geometry_msgs::Point tail;
  geometry_msgs::Point head;

  head.x = x;
  head.y = y;
  head.z = 0.0;

  tail.x = x;
  tail.y = y;
  tail.z = 1.0;

  marker.points.push_back(tail);
  marker.points.push_back(head);

  marker_publisher_.publish(marker);
}

bool Scooper::parsePositionFile(std::string filename)
{
  FILE* file = fopen(filename.c_str(), "r");
  char sTemp[1024];
  std::vector<float> v(7,0);

  if(file == NULL)
  {
    ROS_ERROR("ERROR: unable to open the file. Exiting.");
    return false;
  }

  if(fscanf(file,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1.");

  while(!feof(file) && strlen(sTemp) != 0)
  {
    if(strcmp(sTemp, "tucked_scooper") == 0)
    {
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing tucked_scooper pose.");
      tucked_scooper_ = v;
    }
    else if(strcmp(sTemp, "tucked_bucket") == 0)
    {
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing tucked_bucket pose.");
      tucked_bucket_ = v;
    }
    else if(strcmp(sTemp, "start_scooper") == 0)
    {
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing start_scooper pose.");
      start_scooper_ = v;
    }
    else if(strcmp(sTemp, "drop_scooper") == 0)
    {
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing drop_scooper pose.");
      drop_scooper_ = v;
    }
    else if(strcmp(sTemp, "drop_bucket") == 0)
    {
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing drop_scooper pose.");
      drop_bucket_ = v;
    }
    else if(strcmp(sTemp, "floor_scooper") == 0)
    {
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing start_scooper pose.");
      floor_scooper_ = v;
    }
    else
      ROS_ERROR("Can't parse: %s", sTemp);
    if(fscanf(file,"%s",sTemp) < 1)
      ROS_WARN("Parsed string has length < 1. (%s)", sTemp);
  }
  return true;
}

bool Scooper::scoopService(poop_scoop::Scooper::Request &req, poop_scoop::Scooper::Response &res)
{
  res.result = 0;
  if(req.action.compare("scoop") == 0) {
    ROS_INFO("Received a request to scoop a poop.");
    adjust_flag = false;
    scoopPoop();
  }
  else if(req.action.compare("start") == 0) {
    startScoop();
  }
  else if(req.action.compare("drop") == 0) {
    dropPoop();
  }
  else if(req.action.compare("tuck") == 0) {
    returnScoop();
  }
  else if(req.action.compare("floor") == 0) {
    rarm_->sendArmToConfiguration(floor_scooper_, 2.0);
  }
  else if(req.action.compare("adjusted_scoop") == 0) {
    ROS_INFO("Received a request to scoop a poop at an adjusted location.");
    adjust_x = req.poop_location.pose.position.x;
    adjust_y = req.poop_location.pose.position.y;
    adjust_flag = true;
    scoopPoop();
  }
  else {
    res.result = 0;
  }
  return true;
}

void Scooper::openScooper()
{
  if (use_gripper_sensors_flag_)
  {
    std_srvs::Empty empty;
    SensorGripperService_OpenScoop_.call(empty);
  }
  else
    rgripper_->close();
}

void Scooper::closeScooper()
{
  if (use_gripper_sensors_flag_){
    std_srvs::Empty empty;
    SensorGripperService_CloseScoop_.call(empty);
  }
  else
    rgripper_->open();
}

void Scooper::performInitialScooperGrab()
{
  // Move to prepare to lower the scooper, and open it.
  ROS_INFO("INITAL SCOOP GRAB: Moving to the start_scooper location.");
  rarm_->sendArmToConfiguration(start_scooper_, 2.0);
  sleep(2.0);

  if (use_gripper_sensors_flag_)
  {
    std_srvs::Empty empty;
    ROS_INFO("INITIAL SCOOP GRAB WITH SENSOR: Place scoop in hand within 5 seconds.");
    SensorGripperService_OpenGripper_.call(empty);
    sleep(5.0);
    ROS_INFO("INITIAL SCOOP GRAB: Grabbing scoop... I hope you put it in my hand!");
    SensorGripperService_GrabScoop_.call(empty);
  }
  else
  {
    ROS_INFO("INITIAL SCOOP GRAB: Place scoop in hand within 5 seconds.");
    closeScooper();
    sleep(5.0);
    ROS_INFO("INITIAL SCOOP GRAB: Grabbing scoop... I hope you put it in my hand!");
    openScooper();
  }

}

void Scooper::startScoop()
{
	// Move to prepare to lower the scooper, and open it.
	ROS_INFO("Moving to the start_scooper location.");
	rarm_->sendArmToConfiguration(start_scooper_, 2.0);
	openScooper();
	sleep(2);
	//  openScooper();
	//  sleep(0.5);
}

void Scooper::lowerScoop()
{
	// Get the default desired pose. IS THIS NEEDED?
	std::vector<double> floor_scooper_pose;
	std::vector<double> floor_scooper_config;
	floor_scooper_config.assign(floor_scooper_.begin(), floor_scooper_.end());
	rarm_->performFK(floor_scooper_config, "/r_gripper_l_finger_tip_link", floor_scooper_pose);
//	ROS_INFO("[scooper] Original location: %f, %f, %f", floor_scooper_pose[0], floor_scooper_pose[1], floor_scooper_pose[2]); 
//	ROS_INFO("[scooper] Original orientation: %f, %f, %f, %f", floor_scooper_pose[3], floor_scooper_pose[4], floor_scooper_pose[5], floor_scooper_pose[6]);
	placeMarker(floor_scooper_pose[0], floor_scooper_pose[1], 1.0, 1.0, 0.0);

	// Move to the floor and close the scooper
	ROS_INFO("Moving scooper to the floor.");
	if(!adjust_flag)
	{
//		floor_scooper_pose.clear();
//		// location
//		floor_scooper_pose.push_back( 0.58);  // x
//		floor_scooper_pose.push_back(-0.175);  // y
//		floor_scooper_pose.push_back( 0.69);  // z
//		// orientation 
//		floor_scooper_pose.push_back( -1.57 );   // r
//		floor_scooper_pose.push_back( 0 );   // p
//		floor_scooper_pose.push_back( 1.57 );   // y
//
//		double pose_array[6];
//		for (int i = 0; i < 6; i++) { pose_array[i] = floor_scooper_pose[i]; }
//		rarm_->sendArmToPose(pose_array, 2.0);

		rarm_->sendArmToConfiguration(floor_scooper_, 2.0);
	}
	else
	{
    floor_scooper_pose.clear();
    // location
    floor_scooper_pose.push_back( adjust_x);     // x
    floor_scooper_pose.push_back( adjust_y-0.175);     // y
    floor_scooper_pose.push_back( 0.69);    // z
    // orientation -0.540, -0.487, 0.452, 0.517
    floor_scooper_pose.push_back( -1.57 );   // r
    floor_scooper_pose.push_back(  0.0 );       // p
    floor_scooper_pose.push_back(  1.57 );   // y

	  ROS_INFO("Sending arm to adjusted location: %f, %f, %f", floor_scooper_pose[0], floor_scooper_pose[1], floor_scooper_pose[2]);
	  ROS_INFO("Sending arm to adjusted orientation: %f, %f, %f, %f", floor_scooper_pose[3], floor_scooper_pose[4], floor_scooper_pose[5], floor_scooper_pose[6]);
	  placeMarker(floor_scooper_pose[0], floor_scooper_pose[1], 1.0, 0.0, 1.0);

	  double pose_array[6];
	  for (int i = 0; i < 7; i++) { pose_array[i] = floor_scooper_pose[i]; }

	  rarm_->sendArmToPose(pose_array, 2.0);
	}

	sleep(2.5); //2.5
  
//  floor_scooper_config.clear();
//  rarm_->getCurrentArmConfiguration(floor_scooper_config);
//	ROS_INFO("[scooper] joint angles: %f %f %f %f %f %f %f", floor_scooper_config[0], floor_scooper_config[1], floor_scooper_config[2], floor_scooper_config[3], floor_scooper_config[4], floor_scooper_config[5], floor_scooper_config[6]);

	closeScooper();
	sleep(1.5); // 1.5
}

void Scooper::raiseScoop()
{
	// Lift the scooper up to drop the poop in the bucket.
	ROS_INFO("Moving bucket and the scooper to drop poop in bucket.");
	rarm_->sendArmToConfiguration(drop_scooper_, 3.0);
	larm_->sendArmToConfiguration(drop_bucket_, 3.0);
	sleep(2.5); //3.5
}

void Scooper::dropPoop()
{
	  head_.lookAt("base_link", 1.0, 0.25, 0.0);
	  //Drop the poop in the bucket
	  ROS_INFO("Dropping the poop... is it in there?");
	  openScooper();
	  sleep(2.0);
}

void Scooper::returnScoop()
{
//  head_.lookAt("base_link", 1.0, 0.0, 1.0);
  // Close the scoop and tuck arms.
	ROS_INFO("Release poop into the bucket! Tucking arms.");
	closeScooper();
	larm_->sendArmToConfiguration(tucked_bucket_, 2.0);
	rarm_->sendArmToConfiguration(tucked_scooper_, 2.0);
	sleep(2);
}

void Scooper::scoopPoop()
{
  startScoop();

  lowerScoop();

  raiseScoop();

  //dropPoop();
  //returnScoop();
}

int Scooper::run()
{
  ros::spin();
  return 1;
}

void Scooper::printDebugInfo()
{
  printf("tucked_bucket: ");
  for(size_t i = 0; i < tucked_bucket_.size(); ++i)
    printf("%0.3f ", tucked_bucket_[i]);
  printf("\n");

  printf("tucked_scooper: ");
  for(size_t i = 0; i < tucked_scooper_.size(); ++i)
    printf("%0.3f ", tucked_scooper_[i]);
  printf("\n");

  printf("start_scooper: ");
  for(size_t i = 0; i < start_scooper_.size(); ++i)
    printf("%0.3f ", start_scooper_[i]);
  printf("\n");

  printf("floor_scooper: ");
  for(size_t i = 0; i < floor_scooper_.size(); ++i)
    printf("%0.3f ", floor_scooper_[i]);
  printf("\n");
  printf("drop_scooper: ");
  for(size_t i = 0; i < drop_scooper_.size(); ++i)
    printf("%0.3f ", drop_scooper_[i]);
  printf("\n");
  printf("drop_bucket: ");
  for(size_t i = 0; i < drop_bucket_.size(); ++i)
    printf("%0.3f ", drop_bucket_[i]);
  printf("\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scooper_service");
  Scooper scooper;

  //scooper.scoopPoop();

  return scooper.run();
}

