#!/usr/bin/env python

import roslib; roslib.load_manifest('poop_scoop')

from geometry_msgs.msg import PointStamped, Point32
import rospy
from rospy import logerr, loginfo, Time
from sensor_msgs.msg import PointCloud
from tf import TransformListener


class PoopTransformer:
    def __init__(self):
        self.subscriber = rospy.Subscriber('/poop_perception', PointCloud, 
                                           self._callback)
        self.transformer = TransformListener()
        self.publisher = rospy.Publisher('/poop_perception_odom_combined',
                                         PointCloud)

    def _callback(self, data):
        self.transformer.waitForTransform('odom_combined', 'map', Time.now(),
                                          rospy.Duration(2))
        new_data = PointCloud()
        new_data.header.stamp = Time.now()
        new_data.header.frame_id = 'odom_combined'
        new_data.points = [self._map_to_odom_combined(p, data.header.stamp)
                           for p in data.points]
        new_data.channels = data.channels
        self.publisher.publish(new_data)
    
    def _map_to_odom_combined(self, point_in_map, stamp):
        """Takes and returns a Point32."""
        ps = PointStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = stamp
        ps.point.x = point_in_map.x
        ps.point.y = point_in_map.y
        ps.point.z = point_in_map.z
        self.transformer.waitForTransform('odom_combined', 'map', stamp,
                                          rospy.Duration(2))
        point_in_odom = self.transformer.transformPoint('odom_combined', ps)
        z = 25 if point_in_map.z == 25 else point_in_map.z
        return Point32(point_in_odom.point.x, point_in_odom.point.y, z)

if __name__ == '__main__':
    rospy.init_node('poop_perception_transformer')
    transformer = PoopTransformer()
    rospy.spin()
