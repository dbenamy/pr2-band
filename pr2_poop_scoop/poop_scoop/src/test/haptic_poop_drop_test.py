#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @author Joe Romano
# Test script for haptic poop drop class



import roslib; roslib.load_manifest('poop_scoop')
import rospy
import sys


# goal message and the result message.
import os
import sys
sys.path.append(os.path.realpath(__file__ + '..'))
import haptic_poop_drop_checker as hpdc

if __name__ == '__main__':
        # Initializes a rospy node 
        rospy.init_node("poop_checker")

        # instantiate the class for poop drop checking
        drop_check = hpdc.HapticPoopDropChecker();

        # start listening for a poop drop now!
        drop_check.startDetection();
        
        print "Started listening for contact .. going to sleep";
        # do some other stuff here like opening the scooper
        rospy.sleep(10.);

        # abort the listening and check if we felt a poop
        if(drop_check.abortDetection()):
            print "POOPIES!!!";
        else:
            print "no poopies :-(";

