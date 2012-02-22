#! /usr/bin/env python

# @author: Joe Romano
# Class to wrap the calls to the pr2_gripper_sensor_action controllers
# for the pr2_poop_scoop project

import roslib; roslib.load_manifest('poop_scoop')
import rospy
import actionlib

# import all the messages from the pr2_gripper_sensor packages
from pr2_gripper_sensor_msgs.msg import *
from pr2_controllers_msgs.msg import *
from std_srvs.srv import *

class GripperScooperController:
    
   # constructor
   def __init__(self):
    # create and connect to the actionlib server
    self.force_client = actionlib.SimpleActionClient('r_gripper_sensor_controller/force_servo', PR2GripperForceServoAction);
    self.contact_client = actionlib.SimpleActionClient('r_gripper_sensor_controller/find_contact', PR2GripperFindContactAction);
    self.action_client = actionlib.SimpleActionClient('r_gripper_sensor_controller/gripper_action', Pr2GripperCommandAction);
    rospy.loginfo("waiting for r_gripper_sensor_action/gripper_action and force_servo servers");
    self.force_client.wait_for_server();
    self.action_client.wait_for_server();
    self.contact_client.wait_for_server();
    rospy.loginfo("Connected to r_gripper action & force_servo servers!");
    
    rospy.loginfo("waiting for r_gripper_sensor_controller/zero_fingertip_sensors");
    rospy.wait_for_service('r_gripper_sensor_controller/zero_fingertip_sensors');
    bgnd_sub = rospy.ServiceProxy('r_gripper_sensor_controller/zero_fingertip_sensors', Empty)
    rospy.loginfo("done with controller init");
    #rospy.loginfo("calling bgnd subtractions");
    #try:
    #   bgnd_sub();
    #except rospy.ServiceException, e:
    #   print "Service call failed: %s"%e
       

   # method to open the scooper (close gripper) 
   # this is a blocking method
   #
   # timeout is the time to give up waiting for the gripper to open and return
   # if a small timeout is given, method will return and gripper will still
   # continue to attempt to reach setpoint
   def open_scoop(self,timeout):
       fgoal = PR2GripperForceServoGoal();
       # servo the gripper to this force (N)
       grip_force = 55.0;
       fgoal.command.fingertip_force = grip_force;
       # send off our goal to the server
       self.force_client.send_goal(fgoal);
       rospy.loginfo("Closing scooper by servoing the Gripper to %f N",grip_force);
       self.force_client.wait_for_result(rospy.Duration.from_sec(timeout));      


   # method to close the scooper (open gripper) on a poop
   # returns true if closed all the way, false otherwise
   #
   # this is a blocking method
   # timeout is the time to give up waiting for the gripper to open (try ~10s)
   # even after return gripper will continue to achieve force setpoint
   def close_scoop(self,timeout):
       fgoal = PR2GripperForceServoGoal();
       # servo the gripper to this force (N)
       grip_force = 25.0;
       fgoal.command.fingertip_force = grip_force;
       # send off our goal to the server
       self.force_client.send_goal(fgoal);
       rospy.loginfo("Closing scooper by servoing the Gripper to %f N",grip_force);
       self.force_client.wait_for_result(rospy.Duration.from_sec(timeout));



   # convenience method to open the robot's gripper all the way, to put in the scoop
   def open_gripper(self):
       goal = Pr2GripperCommandGoal();
       # open the gripper to 0.1
       goal.command.position = 0.1;
       goal.command.max_effort = -1.0;
       self.action_client.send_goal(goal);
       self.action_client.wait_for_result();



   # convenience method to close the gripper down on the scoop
   def grab_scoop(self):
       cgoal = PR2GripperFindContactGoal();
       cgoal.command.contact_conditions = cgoal.command.BOTH;  # close until both fingers contact
       cgoal.command.zero_fingertip_sensors = False;   # zero fingertip sensor values before moving
       self.contact_client.send_goal(cgoal);
       self.contact_client.wait_for_result();
