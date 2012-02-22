#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @author Joe Romano
# Test script for haptic poop drop class



import roslib; roslib.load_manifest('poop_scoop')
import rospy
import os
import sys
sys.path.append(os.path.realpath(__file__ + '..'))

# import the python class for gripper control
import gripper_scooper_controller as gsc

if __name__ == '__main__':
        # Initializes a rospy node 
        rospy.init_node("scoop_checker")

        # instantiate the class for poop drop checking
        mygsc = gsc.GripperScooperController();

	# open the gripper to put the scooper in
	mygsc.open_gripper();
        rospy.sleep(5.);  # wait for the person to put in the scooper

	# grab the scoop with the gripper
	mygsc.grab_scoop();

        # open the scooper, waiting 10s for it to get to the right force
        mygsc.open_scoop(10.0)

	# do some stuff with the robot ....
        rospy.sleep(5.);
	
        # close the scooper, waiting 10s for it to get to the right force
        mygsc.close_scoop(10.0)

	# do some stuff with the robot
        rospy.sleep(5.);

