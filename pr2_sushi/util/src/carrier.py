#!/usr/bin/env python

"""This module handles 2 arm coordination and other tasks associates with
lifting and carrying.

"""


from objdb import get_pregrasp_poses, get_grasp_poses, get_carry_pose


# I was thinking about making this a separate node so it could keep state across
# runs of main, but that's probably more trouble than it's worth.

#_NODE_NAME = 'amcl_pose_gossip'
#_SUB_TOPIC = 'amcl'

LEFT_HAND = 'LEFT_HAND'
RIGHT_HAND = 'RIGHT_HAND'
BOTH_HANDS = 'BOTH_HANDS'


def pregrasp(self, obj, hand):
    poses = [pose_obj_to_map(obj, x) for x in get_pregrasp_poses(obj.type)]
    best_pose = best_pose(obj, poses)
    wrist_to(best_pose, hand)


def grasp(self, obj, hand):
    poses = [pose_obj_to_map(obj, x) for x in get_grasp_poses(obj.type)]
    best_pose = best_pose(obj, poses)
    wrist_to(best_pose, hand)
    close_gripper(hand)


def lift(self, obj, hand):
    wrist_pose
    wrist_roll_base, wrist_pitch_base = 0, 0 # TODO get current
    # TODO read these from file:
    if hand == LEFT_HAND:
        wrist_position_base = [1, 1, 1]
        wrist_yaw_base = 1
    elif hand == RIGHT_HAND:
        wrist_position_base = [1, -1, 1]
        wrist_yaw_base = -1
    wrist_pose_map = pose_base_to_map(wrist_position_base[0],
                                      wrist_position_base[1],
                                      wrist_position_base[2],
                                      wrist_roll_base, wrist_pitch_pase,
                                      wrist_yaw_base)
    wrist_to(pose_map, hand)
    # TODO haptic check


def pick_up(self, obj, hand):
    pregrasp(obj, hand)
    grasp(obj, hand)
    lift(obj, hand)


def put_down_obj_at(self, x_map, y_map, z_map, hand):
    # TODO given the dest position, figure out where the wrist should be
    wrist_to(x_map, y_map, z_map, hand)
    open_gripper()


def best_pose(obj, manip_poses):
    """Takes an object and a list of possible manipulation poses in the map
    frame, and returns the best pose.
    
    """
    # TODO
    return manip_poses[0]


def wrist_to(pose, hand):
    """Takes a MapFramePose."""
    pass # TODO


def open_gripper(hand):
    pass # TODO


def close_gripper(hand):
    pass # TODO


def wrist_pose(hand):
    """Returns the current wrist poses as a MapPose."""
    pass # TODO


"""
from haptic_poop_drop_checker import HapticPoopDropChecker


#RESENSE_TIMEOUT = 10.0


class Scooper():
    def __init__(self):
        SERVICE = 'scoop_poop'
        loginfo("Waiting for %s service." % SERVICE)
#        rospy.wait_for_service(SERVICE)
        self._scooper = rospy.ServiceProxy(SERVICE, poop_scoop.srv.Scooper)
        loginfo("Connected to %s service." % SERVICE)
        self.dropDetector = HapticPoopDropChecker()

    def scoop(self):
        loginfo("Requesting scoop")
        self._scooper(action='scoop')
        
        self.dropDetector.startDetection()
        self._scooper(action='drop')
       
        if self.dropDetector.checkDetection() == False:
            self.speak("Ooops. I'll try again.")
            self.dropDetector.abortDetection()
        else:
            self.speak("Poop scooped.")
       
        self._scooper(action='tuck')

    def start(self):
        self._scooper(action='start')
    
    def floor(self):
        self._scooper(action='floor')

    def tuck(self):
        self._scooper(action='tuck')     

"""

#def main():
#    loginfo("%s starting up." % NODE_NAME)
#    rospy.init_node(NODE_NAME)
#    sleep(1)
#    rospy.Subscriber(SUB_TOPIC, PoseWithCovarianceStamped, pose_callback)
#
#    while not rospy.is_shutdown():
#
#if __name__ == '__main__':
#    try:
#        main()
#    except rospy.ROSInterruptException:
#        pass