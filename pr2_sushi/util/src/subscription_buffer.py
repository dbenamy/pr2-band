from threading import Lock

import roslib; roslib.load_manifest('poop_scoop')
import rospy
from rospy import loginfo, logwarn

class SubscriptionBuffer:
    def __init__(self, topic, msg, blocking=True):
        self._topic = topic
        self._blocking = blocking
        self._data = None
        self._lock = Lock()
        self._sub = rospy.Subscriber(topic, msg, self._callback)
        loginfo("SubscriptionBuffer listening on %s for %s." % (topic, msg))
    
    def get_last(self):
        self._lock.acquire()
        try:
            if self._blocking:
                while self._data is None and not rospy.is_shutdown():
                    self._lock.release()
                    logwarn("SubscriptionBuffer is blocked waiting for data "
                            "on topic '%s'" % self._topic)
                    rospy.sleep(1)
                    self._lock.acquire()
                return self._data
            else:
                raise NotImplemented()
        finally:
            self._lock.release()
    
    def _callback(self, data):
        self._lock.acquire()
        try:
            self._data = data
        finally:
            self._lock.release()

if __name__ == '__main__':
    from sensor_msgs.msg import PointCloud
    rospy.init_node('test_subscription_buffer')
    buffer = SubscriptionBuffer('poop_perception', PointCloud, blocking=True)
    print buffer.get_last()
