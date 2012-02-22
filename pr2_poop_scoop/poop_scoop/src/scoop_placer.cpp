#include <poop_scoop/scoop_placer.h>

ScoopPlacer::ScoopPlacer() : ph_("~")
{
	impeded_joint_sub_ = nh_.subscribe("/impeded_joint_detector/right_arm", 1, &ScoopPlacer::ImpededJointCallback, this);
	scoop_placer_service_ = nh_.advertiseService("/place_scoop", &ScoopPlacer::scoopPlacerService, this);

	ph_.param("lower_distance", lowerDistance, 1.0);

  //larm_ = new Arm(std::string("left"));
  rarm_ = new Arm(std::string("right"));

  sleep(1);
  
  ROS_INFO("ScoopPlacer is initialized.");
}

ScoopPlacer::~ScoopPlacer()
{
  //delete larm_;
  delete rarm_;
}

void ScoopPlacer::ImpededJointCallback(const std_msgs::Int8::ConstPtr& msg)
{
	rarm_->stopArm();
	ROS_INFO("Detected impedance on Right Arm.");
}

bool ScoopPlacer::scoopPlacerService(poop_scoop::ScoopPlacerSrv::Request &req, poop_scoop::ScoopPlacerSrv::Response &res)
{
  if(req.action.compare("place") == 0)
    ROS_INFO("Received a request to reactively place the scooper.");

  lowerScoop();

  res.result = 1;

  return true;
}

int ScoopPlacer::run()
{
  ros::spin();
  return 1;
}

void ScoopPlacer::lowerScoop()
{
  ROS_INFO("Lowering scoop.");

	// Figure out where the scooper is now (cartesian).
	std::vector<double> current_pose;
	rarm_->getCurrentArmPose(current_pose);

	// Lower position by some amount.
	current_pose[1] -= 1.0;

	double cpose_array[7];
	for (int i = 0; i < 7; i++)
		cpose_array[i] = current_pose[i];

	rarm_->sendArmToPoseQuaternion(cpose_array, 5);
	ROS_INFO("Scoop lowering goal sent.");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scoop_placer_service");
  ScoopPlacer scoopPlacer;

  scoopPlacer.lowerScoop();
  sleep(10);

  return scoopPlacer.run();
}

