import roslib; roslib.load_manifest('util')
from geometry_msgs.msg import Point
import rospy
from rospy import logerr, loginfo, ServiceProxy
from util.srv import PointHead


class Head():
    def __init__(self):
        SERVICE = 'point_head'
        loginfo("Waiting for %s service." % SERVICE)
#        rospy.wait_for_service(SERVICE)
        self._head = ServiceProxy(SERVICE, PointHead)
        loginfo("Connected to %s service." % SERVICE)
    
    def look_far(self):
        loginfo("Looking far away.")
        self._head(location=Point(1.0, 0.25, 0))
    
    def look_near(self):
        loginfo("Looking near by.")
        self._head(location=Point(0.4, 0, 0))
