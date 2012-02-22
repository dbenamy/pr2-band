#! /usr/bin/env python

# @author: Joe Romano
# Class to wrap the calls to the pr2_gripper_sensor_action Event Detector
# actionlib server for the pr2_poop_scoop project

import roslib; roslib.load_manifest('poop_scoop')
import rospy
import actionlib

# import all the messages from the pr2_gripper_sensor packages
from pr2_gripper_sensor_msgs.msg import *

class HapticPoopDropChecker:
    
   # constructor
   def __init__(self):
    # create and connect to the actionlib server
    self.client = actionlib.SimpleActionClient('l_gripper_sensor_controller/event_detector', PR2GripperEventDetectorAction);
    rospy.loginfo("waiting for l_gripper_sensor_action/event_detector");
    self.client.wait_for_server();
    rospy.loginfo("Connected to l_gripper event detection server!");
    self.poop_detected =  False;

   # method to call when you are done, returns success (poop detected) or not 
   def abortDetection(self):
       # check what the state was before we cancel
       status = self.checkDetection();
       # cancel the goal;
       self.client.cancel_goal();
       rospy.loginfo("Canceled poop drop detection goal!");
       # return the last poop detection status
       return status;

   # method to call to start listening for a poop falling
   def startDetection(self):
       self.poop_detected =  False;
       goal = PR2GripperEventDetectorGoal();
       # trigger on just acceleration spikes
       goal.command.trigger_conditions = goal.command.ACC;
       # trigger on 4 m/s^2 of acceleration
       goal.command.acceleration_trigger_magnitude = 3.0;
       # send off our goal to the server
       self.client.send_goal(goal);
       rospy.loginfo("started detection!");

   # method to call at any time to get status of detection (poop detected)
   def checkDetection(self):
       state = self.client.get_state();
       if(state == actionlib.GoalStatus.SUCCEEDED):
           self.poop_detected = True;
       else:
           self.poop_detected = False;
       return self.poop_detected;


