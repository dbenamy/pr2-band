#!/usr/bin/env python


# I was thinking about making this a separate node so it could keep state across
# runs of main, but that's probably more trouble than it's worth.

#_NODE_NAME = 'amcl_pose_gossip'
#_SUB_TOPIC = 'amcl'

LEFT_HAND = 'LEFT_HAND'
RIGHT_HAND = 'RIGHT_HAND'
BOTH_HANDS = 'BOTH_HANDS'


class Carrier:
    """This class handles 2 arm coordination and other tasks associates with
    lifting and carrying.
    
    """
    def pregrasp(self, obj, hand):
        pass
    
    def grasp(self, obj, hand):
        pass
    
    def lift(self, obj, hand):
        pass
    
    def pick_up(self, obj, hand):
        pass
    
    def put_down_obj_at(self, x_map, y_map, z_map, hand,
                        grasp_height=None):
        self.move_tip_to(x_map, y_map, z_map + grasp_height) # TODO which way is z? Should it be -?
        self.open_gripper()


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