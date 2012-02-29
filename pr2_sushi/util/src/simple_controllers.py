from math import atan2, pi, sqrt
import sys
from time import sleep
from os import system

import roslib; roslib.load_manifest('util')
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import (Point, PointStamped, Point32, Pose, PoseStamped,
                               PoseWithCovariance, PoseWithCovarianceStamped,
                               Quaternion, Vector3)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
from rospy import logerr, loginfo, Time
from sensor_msgs.msg import ChannelFloat32, PointCloud
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from util.srv import PointHead
from util.utiltypes import MapPose


class Head():
    def __init__(self):
        SERVICE = 'point_head'
        loginfo("Waiting for %s service." % SERVICE)
#        rospy.wait_for_service(SERVICE)
        self._head = rospy.ServiceProxy(SERVICE, PointHead)
        loginfo("Connected to %s service." % SERVICE)
    
    def look_far(self):
        loginfo("Looking far away.")
        self._head(location=Point(1.0, 0.25, 0))
    
    def look_near(self):
        loginfo("Looking near by.")
        self._head(location=Point(0.4, 0, 0))


class Base():
    # TODO See if this can use SubscriptionBuffer to always be able to get
    # the current base pose instead of pose_gossip to simplify things.
    
    def __init__(self, start_x, start_y, start_yaw):
        self.initial_pose_pub = rospy.Publisher('initialpose',
                                                PoseWithCovarianceStamped)
        self.move_base_ac = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_ac.wait_for_server()
        self.amcl_pose_sub = rospy.Subscriber('pose_gossip',
                                              PoseWithCovarianceStamped,
                                              self._amcl_pose_callback)
        self.move_base_status_sub = rospy.Subscriber('move_base/status',
                                                      GoalStatusArray,
                                                      self._move_base_status_callback)
        self._start_x = start_x
        self._start_y = start_y
        self._start_yaw = start_yaw
        self._x_map = None
        self._y_map = None
        self._yaw_map = None
        self._moving = False
    
    def get_x_map(self):
        self._wait_for_base_pose()
        return self._x_map
    
    def get_y_map(self):
        self._wait_for_base_pose()
        return self._y_map
    
    def get_yaw_map(self):
        self._wait_for_base_pose()
        return self._yaw_map
    
    def get_pose(self):
        self._wait_for_base_pose()
        return MapPose(self._x_map, self._y_map, 0, 0, 0, self._y_map)
    
    def _wait_for_base_pose(self):
        while self._x_map is None and not rospy.is_shutdown():
            logerr("Poop scoop behavior hasn't gotten base pose data. "
                   "Please move the base.")
            sleep(2)
    
    def _amcl_pose_callback(self, data):
        self._x_map = data.pose.pose.position.x
        self._y_map = data.pose.pose.position.y
        q = data.pose.pose.orientation
        angles = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._yaw_map = angles[2]
    
    def _move_base_status_callback(self, data):
      if data.status_list and data.status_list[0].status == 3: # SUCCEEDED
        self._at_goal = True
      else:
        self._at_goal = False
      if (not data.status_list) or (not data.status_list[0].status == 1): # ACTIVE
        self._moving = False
      else:
        self._moving = True

    def reset_pose(self):
        """Sets the current pose to the start pose. Doesn't move the robot."""
        loginfo("Resetting pose.")
        req = PoseWithCovarianceStamped()
        req.header = Header(stamp=Time.now(), frame_id='/map')
        req.pose.pose = self._x_y_yaw_to_pose(self._start_x, self._start_y,
                                              self._start_yaw)
        req.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.initial_pose_pub.publish(req)
        self.go_to_start()
    
    def go_to(self, x_map, y_map, yaw_map):
        """Moves to the given x, y, and yaw in map coordinates."""
        loginfo("Going to pose x = %s, y = %s, yaw = %s." %
                (x_map, y_map, yaw_map))
        goal = MoveBaseGoal()
        goal.target_pose.header = Header(stamp=Time.now(), frame_id = '/map')
        goal.target_pose.pose = self._x_y_yaw_to_pose(x_map, y_map, yaw_map)
        self.move_base_ac.send_goal(goal)
        loginfo("Send goal to move base. Waiting for result.")
        self.move_base_ac.wait_for_result()
        #loginfo("Got result: %s" % self.move_base_ac.get_result())
        #loginfo("Pose: %s, %s, %s" %
        #        (self.get_x_map(), self.get_y_map(), self.get_yaw_map()))
        sleep(1)
        loginfo("At Goal: %i", self._at_goal)
        return self._at_goal

    def go_to_pose(self, pose):
        """Takes a MapPose."""
        assert(issubclass(pose.__class__, MapPose))
        return self.go_to(pose.x.val, pose.y.val, pose.yaw.val)
    
    def go_to_start(self):
        self.go_to(self._start_x, self._start_y, self._start_yaw)
    
    def _x_y_yaw_to_pose(self, x, y, yaw):
        position = Point(x, y, 0)
        orientation = Quaternion()
        q = quaternion_from_euler(0, 0, yaw)
        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]
        return Pose(position=position, orientation=orientation)


def speak(text):
    cmd = "echo \"" + text + "\" | festival --tts"
    system(cmd)