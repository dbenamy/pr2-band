/*
 * impeded_joint_detector.h
 *
 *  Created on: Aug 6, 2011
 *      Author: wmcmahan
 */

#ifndef IMPEDED_JOINT_DETECTOR_H_
#define IMPEDED_JOINT_DETECTOR_H_

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <std_msgs/Int8.h>

class ImpededJointDetector{

	public:
		ImpededJointDetector();
		~ImpededJointDetector(){ };

		void RArmCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg);
		void LArmCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle ph_;	// private handle for parameter server
	  ros::Subscriber rsub_;
	  ros::Subscriber lsub_;
	  ros::Publisher rpub_;
	  ros::Publisher lpub_;

	  double rlim[7];
	  double llim[7];

	  std_msgs::Int8 rstatus;
	  std_msgs::Int8 lstatus;
};


#endif /* IMPEDED_JOINT_DETECTOR_H_ */
