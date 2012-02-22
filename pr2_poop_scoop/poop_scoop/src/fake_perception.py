#!/usr/bin/env python

from math import atan2, sqrt
import sys
from time import sleep

import roslib; roslib.load_manifest('poop_scoop')

from actionlib import SimpleActionClient
from geometry_msgs.msg import (Point, PointStamped, Point32, Pose, PoseStamped,
                               PoseWithCovariance, PoseWithCovarianceStamped,
                               Quaternion)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
from rospy import logerr, loginfo, Time
from sensor_msgs.msg import ChannelFloat32, PointCloud
from std_msgs.msg import Header
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import poop_scoop.srv
from subscription_buffer import SubscriptionBuffer


class PoopPublisher:
    def __init__(self):
        self.transformer = TransformListener()
        self.publisher = rospy.Publisher('/poop_perception', PointCloud)
    
    def publish_poops_to_cost_map(self, poops):
        """Always publishes 30 poops. Keeps unused ones at map position 25, 25."""
        NUM_POOPS = 30
        poops_with_z = [(p[0], p[1], 0) for p in poops]
        unused_poops = [(25, 25, 25)] * (NUM_POOPS - len(poops))
        poop_positions = poops_with_z + unused_poops
        #print poop_positions
        
        point_cloud = PointCloud()
        # cost map doesn't like the map coordinate frame
        point_cloud.header = Header(stamp=Time.now(), frame_id='odom_combined')
        point_cloud.points = [self._map_to_odom_combined(pos)
                              for pos in poop_positions]
        #print 'new points', point_cloud.points
        point_cloud.channels = [
            ChannelFloat32(name='intensities', values=[2000] * NUM_POOPS),
            ChannelFloat32(name='index', values=[0] * NUM_POOPS), #values=range(NUM_POOPS)),
            ChannelFloat32(name='distances', values=[2] * NUM_POOPS),
            ChannelFloat32(name='stamps', values=[0.002] * NUM_POOPS),
        ]
        self.publisher.publish(point_cloud)

    def _map_to_odom_combined(self, pos):
        """Transforms (x, y, 0) to a Point32."""
        point_in_map = PointStamped()
        point_in_map.header.frame_id = 'map'
        point_in_map.header.stamp = Time.now()
        point_in_map.point.x = pos[0]
        point_in_map.point.y = pos[1]
        self.transformer.waitForTransform('odom_combined', 'map', Time.now(),
                                          rospy.Duration(2))
        point_in_odom = self.transformer.transformPoint('odom_combined',
                                                        point_in_map)
        z = 0 if pos[2] == 0 else 25
        return Point32(point_in_odom.point.x, point_in_odom.point.y, z)


if __name__ == '__main__':
    rospy.init_node('fake_perception')
    publisher = PoopPublisher()
    sleep(1)
    
    hall1_poops = [ # not in distance order
        (5.644, 3.173),
        (6.065, 1.9),
        (6.620, 4.657),
    ]
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        loginfo("Publishing fake poop cloud.")
        publisher.publish_poops_to_cost_map(hall1_poops)
        rate.sleep()
