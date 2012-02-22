#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @author Joe Romano
# Test script for haptic poop drop class



import roslib; roslib.load_manifest('poop_scoop')
import rospy
import os
import sys
sys.path.append(os.path.realpath(__file__ + '.'))

from std_srvs.srv import *

# import the python class for gripper control
import gripper_scooper_controller as gsc


def grab_scoop(req):
      global mygsc;
      mygsc.grab_scoop();
      return EmptyResponse();


def open_gripper(req):
      global mygsc;
      mygsc.open_gripper();
      return EmptyResponse();

def close_scoop(req):
      global mygsc;
      mygsc.close_scoop(1.0);
      return EmptyResponse();


def open_scoop(req):
      global mygsc;
      mygsc.open_scoop(1.0);
      return EmptyResponse();

if __name__ == '__main__':
        # Initializes a rospy node 
        rospy.init_node("gripper_scooper_controller_service")
        global mygsc;
        mygsc = gsc.GripperScooperController();

	s = rospy.Service('open_scoop', Empty, open_scoop);
	s = rospy.Service('close_scoop', Empty, close_scoop);
	s = rospy.Service('open_gripper', Empty, open_gripper);
	s = rospy.Service('grab_scoop', Empty, grab_scoop);

	rospy.spin();
