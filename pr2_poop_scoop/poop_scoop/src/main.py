#!/usr/bin/env python

from math import atan2, pi, sqrt
import sys
from time import sleep
from os import system
import roslib; roslib.load_manifest('poop_scoop')

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

import poop_scoop.srv
from subscription_buffer import SubscriptionBuffer

from haptic_poop_drop_checker import HapticPoopDropChecker

RESENSE_TIMEOUT = 10.0
WORKING_DIST_FROM_POOP = 0.58 #0.65  #0.55
STAGE1_OFFSET = 0.15
#START_X = 10
#START_Y = 10
#START_YAW = 0
# Hallway outside lab
#START_X = 6.1
#START_Y = 0.2
#START_YAW = 1.544
# Towne
#START_X = 26
#START_Y = 60
#START_YAW = -2.88
START_X = 5.44
START_Y = 0.211
START_YAW = -0.1

# map bounds
MIN_X = 0.45
MAX_X = 1.32 
MIN_Y = -2.75
MAX_Y = -0.84

MAP_CORNERS = [(0.76, -3.08), (1.265, -1.55), (-0.205, -0.45), (-1.5, -1.61)] 
            # [brian_chair, chris_window, fake_door, basestation]

class Scooper():
    def __init__(self):
        SERVICE = 'scoop_poop'
        loginfo("Waiting for %s service." % SERVICE)
#        rospy.wait_for_service(SERVICE)
        self._scooper = rospy.ServiceProxy(SERVICE, poop_scoop.srv.Scooper)
        loginfo("Connected to %s service." % SERVICE)
        self.dropDetector = HapticPoopDropChecker()

    def speak(self, text):
        cmd = "echo \"" + text + "\" | festival --tts"
        system(cmd)

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
    
class Head():
    def __init__(self):
        SERVICE = 'point_head'
        loginfo("Waiting for %s service." % SERVICE)
    #        rospy.wait_for_service(SERVICE)
        self._head = rospy.ServiceProxy(SERVICE, poop_scoop.srv.PointHead)
        loginfo("Connected to %s service." % SERVICE)
        
    def lookFar(self):
        loginfo("Looking for far poops.")
        #self._head(location=Point(1.0, 0, 0))
        self._head(location=Point(1.0, 0.25, 0))
        
    def lookNear(self):
        loginfo("Looking for near poops.")
        self._head(location=Point(0.4, 0, 0))


class Base():
    def __init__(self):
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
        """Sets the current pose to START. Doesn't move the robot."""
        loginfo("Resetting pose.")
        req = PoseWithCovarianceStamped()
        req.header = Header(stamp=Time.now(), frame_id='/map')
        req.pose.pose = self._x_y_yaw_to_pose(START_X, START_Y, START_YAW)
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

    def go_to_start(self):
        self.go_to(START_X, START_Y, START_YAW)
    
    def _x_y_yaw_to_pose(self, x, y, yaw):
        position = Point(x, y, 0)
        orientation = Quaternion()
        q = quaternion_from_euler(0, 0, yaw)
        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]
        return Pose(position=position, orientation=orientation)

def go_to_scoop_poop_at(base, poop_x_map, poop_y_map, offset):
    base_x = base.get_x_map()
    base_y = base.get_y_map()
    x, y, yaw = calc_work_x_y_yaw(base_x, base_y, poop_x_map, poop_y_map, offset)
    rospy.logerr("Calling move base.")
    return base.go_to(x, y, yaw)

def calc_work_x_y_yaw(base_x_map, base_y_map, poop_x_map, poop_y_map, offset):
    # Variable name convention is: b = base, p = poop, w = working position.
    # For example, wp_dx = the dx between the working position and poop
    # position.
    b_x = base_x_map
    b_y = base_y_map
    p_x = poop_x_map
    p_y = poop_y_map
    wp_dist = WORKING_DIST_FROM_POOP+offset
    
    # Find working position
    bp_dx = p_x - b_x
    bp_dy = p_y - b_y
    bp_dist = sqrt(bp_dx**2 + bp_dy**2)
    fraction = wp_dist / bp_dist
    wp_dx = fraction * bp_dx
    wp_dy = fraction * bp_dy
    w_x = p_x - wp_dx
    w_y = p_y - wp_dy
    
    # Find working yaw
    w_yaw = atan2(bp_dy, bp_dx)
    
    return w_x, w_y, w_yaw


"""
def current_position(self):
    self.transformer.waitForTransform('map', 'base_footprint', Time.now())
    base_in_base = PointStamped()
    base_in_base.header.frame_id = 'base_footprint'
    base_in_base.header.stamp = Time.now()
    base_in_map = transformer.transformPoint('map', base_in_base)
    return (current_point.point.x, current_point.point.y)
"""


def dist_between(x1, y1, x2, y2):
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)


def scoop_poops_and_go_back(base, scooper, poops):
    """poops is a list of tuples (map_x, map_y)."""
    for poop in poops:
      if go_to_scoop_poop_at(base, poop[0], poop[1],0):
          scooper.scoop()
#    base.go_to_start()


def get_lowest_2d_poop(points2d):
    poops2d = [(p.x, p.y) for p in points if p.z != 25]
    loginfo("Got the following 2d real poops: %s" % poops2d)
    if len(poops2d) == 0:
        return None
    def get_row(poop2d):
        return poop2d.y
    poops2d.sort(key=get_row)
    loginfo("Poops (map x, map y, distance) sorted by row: %s" % poops2d)
    return poops2d[-1]



def main():
    loginfo("Poop Scoop logic starting up.")
    rospy.init_node('poop_scoop_behavior')
    scooper = Scooper()
    base = Base()
    head = Head()
    sleep(1)
    head.lookNear()
   
    viz_pub = rospy.Publisher("visualization_marker", Marker)

    def visualize_poop(x,y,z,color,frame,ns):
      msg = Marker()
      msg.header = Header(stamp=Time.now(), frame_id=frame)
      #msg.scale = Vector3(x=0.02, y=0.02, z=0.02) # for sphere
      msg.scale = Vector3(x=0.005, y=0.04, z=0.0) # for arrow
      #msg.pose.position = Point(x=x, y=y, z=z)
      #msg.pose.position = Point(x=x, y=y, z=z+0.15) # arrow
      #msg.pose.orientation = Quaternion(x=0, y=0, z=0, w=1) # arrow
      #msg.pose.orientation = Quaternion(x=0.707, y=0, z=0, w=0.707)
      msg.points = [Point(x=x, y=y,z=z+0.15), Point(x=x,y=y,z=z)]
      msg.ns = ns
      msg.id = 0
      msg.action = 0 # add
      #msg.type = 2 # sphere
      msg.type = 0 # arrow
      msg.color = ColorRGBA(r=0, g=0, b=0, a=1)
      if color == 0:
        msg.color.g = 1;
      elif color == 1:
        msg.color.b = 1;
      elif color == 2:
        msg.color.r = 1; 
        msg.color.g = 1; 
      elif color == 3:
        msg.color.g = 1; 
        msg.color.b = 1; 

      #loginfo("Publishing %s marker at %0.3f %0.3f %0.3f",ns,x,y,z)
      viz_pub.publish(msg)

    def visualize_base_ray():
      msg = Marker()
      msg.header = Header(stamp=Time.now(), frame_id="base_footprint")
      msg.scale = Vector3(x=0.005, y=0.0, z=0.0) # only x is used
      msg.pose.position = Point(x=0, y=0, z=0) # arrow
      msg.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
      msg.points = [Point(x=0, y=0,z=0.01), Point(x=WORKING_DIST_FROM_POOP,y=0,z=0.01)]
      msg.ns = "base_ray"
      msg.id = 0
      msg.action = 0 # add
      msg.type = 4 # line strip
      msg.color = ColorRGBA(r=0, g=0, b=0, a=1)
      msg.color.g = 0.5;
      msg.color.b = 1; 
      viz_pub.publish(msg)

    def is_in_bounds(x,y):
      if x < MIN_X or x > MAX_X:
        return False
      if y < MIN_Y or y > MAX_Y: 
        return False
      return True

    # determine if a point is inside a given polygon or not
    # Polygon is a list of (x,y) pairs.
    # http://www.ariel.com.au/a/python-point-int-poly.html
    def point_inside_polygon(x,y):

      poly = MAP_CORNERS #[(0.5, -2.75), (1.28, 1.494), (-0.6,-0.586), (-1.1, -1.4)]
      n = len(poly)
      inside =False

      p1x,p1y = poly[0]
      for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
          if y <= max(p1y,p2y):
            if x <= max(p1x,p2x):
              if p1y != p2y:
                xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
              if p1x == p2x or x <= xinters:
                inside = not inside
        p1x,p1y = p2x,p2y

      return inside

    def speak(text):
      cmd = "echo \"" + text + "\" | festival --tts"
      system(cmd)
      

    mode = sys.argv[1]
    if mode == 'scoop':
        scooper.tuck()
        scooper.scoop()    
    elif mode == 'reset_pose':
        base.reset_pose()
    elif mode == 'go_to':
        x_map = float(sys.argv[2])
        y_map = float(sys.argv[3])
        yaw_map = pi / 2
        if len(sys.argv) >= 5:
            yaw_map = float(sys.argv[4])
        base.go_to(x_map, y_map, yaw_map)
    elif mode == 'go_to_start':
        base.go_to_start()
    elif mode == 'v1':
        POOP = (6.065, 1.9)
        go_to_scoop_poop_at(base, POOP[0], POOP[1],0)
        scooper.scoop()
        base.go_to_start()
    elif mode == 'v2':
        hall1_poops = [
            (6.065, 1.9),
            (5.644, 3.173),
            (6.620, 4.657),
        ]
        scoop_poops_and_go_back(base, scooper, hall1_poops)
    elif mode == 'go_to_poop':
        x_map = float(sys.argv[2])
        y_map = float(sys.argv[3])
        go_to_scoop_poop_at(base, x_map, y_map,0)
    elif mode == 'scoop_poop':
        x_map = float(sys.argv[2])
        y_map = float(sys.argv[3])
        go_to_scoop_poop_at(base, x_map, y_map,0)
        scooper.scoop()
    elif mode == 'v3':
#        hall1_poops = [
#            (5.644, 3.173),
#            (6.065, 1.9),
#            (6.620, 4.657),
#        ]
#        hall2_poops = [
#            (24.7, 59.6),
#            (23.2, 60.06),
#            (22.9, 58.97),
#            (25.2, 58.5),
#        ]
        hall2_poops = [
            (1.357, -0.157),
#            (23.2, 60.06),
        ]
        poop_perception = SubscriptionBuffer('/poo_view', PointCloud,
                                             blocking=True)
        points = poop_perception.get_last().points
        poops = [(p.x, p.y) for p in points if p.z != 25]
        poops = hall2_poops
        loginfo("Got the following FAKE poops: %s" % poops)
        
        base_x = base.get_x_map()
        base_y = base.get_y_map()
        poops_w_dist = [(p[0], p[1], dist_between(p[0], p[1], base_x, base_y))
                        for p in poops]
        def get_dist(poop_w_dist):
            return poop_w_dist[2]
        poops_w_dist.sort(key=get_dist)
        loginfo("Poops (map x, map y, distance) sorted by distance: %s" %
                poops_w_dist)
        
        scoop_poops_and_go_back(base, scooper, poops_w_dist)
    elif mode == 'v4':
        poop_perception = SubscriptionBuffer('/poo_view', PointCloud,
                                             blocking=True)

        speak("Hello! Its time for me to scoop some poop.")

        while not rospy.is_shutdown():
            if base._moving:
              loginfo("\nMOVING\n")
              sleep(1.0)
              continue

            visualize_base_ray()
       
            logerr("\n\n[stage 1] Getting new list of poops.")
            points = poop_perception.get_last().points
            poops = [(p.x, p.y) for p in points if p.z != 25]
            #loginfo("Got the following real poops: %s" % poops)
            if len(poops) == 0:
              loginfo("[stage 1] No poops found.")
              break
            
            base_x = base.get_x_map()
            base_y = base.get_y_map()
            poops_w_dist = [(p[0], p[1],
                             dist_between(p[0], p[1], base_x, base_y))
                            for p in poops]
            def get_dist(poop_w_dist):
                return poop_w_dist[2]
            poops_w_dist.sort(key=get_dist)
            #loginfo("Poops (map x, map y, distance) sorted by distance: %s" %
            #        poops_w_dist)
            closest = poops_w_dist[0]
            if closest[2] > 1.5:
              loginfo("[stage 1] Closest poop is too far.");
              continue
        
            visualize_poop(closest[0],closest[1],0.02,0,"/map","sensed_poop")
            x, y, yaw = calc_work_x_y_yaw(base_x, base_y, closest[0],closest[1],STAGE1_OFFSET)
            visualize_poop(x,y,0.02,3,"/map","projected_scoop")

                        
            #if is_in_bounds(closest[0], closest[1]):
            if not point_inside_polygon(closest[0], closest[1]):
              #speak("Out of bounds.")
              loginfo("[stage 1] Poop location is out of bounds. Ignoring.")
              continue


            logerr("[stage 1] Sending poop to move_base.")
            if go_to_scoop_poop_at(base, closest[0], closest[1], STAGE1_OFFSET):
              visualize_base_ray()
              head.lookNear() 
              sleep(0.5)
              while base._moving:
                loginfo("\n[stage 2] Ignoring perception while I'm moving.\n")
                sleep(1.0)
              sleep(2.0);
              lost_goal_poop = True
              start_resense = rospy.get_time()
              resense_failed = False   # a timeout for resensing
              while lost_goal_poop:
                if rospy.get_time()-start_resense >= RESENSE_TIMEOUT:
                  logerr("[stage 2] TIMED OUT (%0.3f sec).", rospy.get_time()-start_resense)
                  resense_failed = True
                  break

                logerr("[stage 2] Getting new list of poops.")
                points = poop_perception.get_last().points
                poops = [(p.x, p.y) for p in points if p.z != 25]
                #loginfo("Got the following real poops: %s" % poops)
                if len(poops) == 0:
                  continue
            
                base_x = base.get_x_map()
                base_y = base.get_y_map()
                poops_w_dist = [(p[0], p[1],
                             dist_between(p[0], p[1], closest[0], closest[1]))
                            for p in poops]
                def get_dist(poop_w_dist):
                  return poop_w_dist[2]
                poops_w_dist.sort(key=get_dist)
                #loginfo("Poops (map x, map y, distance) sorted by distance: %s" %
                #        poops_w_dist)
                closest2 = poops_w_dist[0]
                visualize_poop(closest2[0],closest2[1],0.02,1,"/map","resensed_poop") 

                x, y, yaw = calc_work_x_y_yaw(base_x, base_y, closest2[0],closest2[1],0)
                visualize_poop(x,y,0.02,2,"/map","reprojected_scoop")

                if closest2[2] > 0.5:
                  loginfo("[stage 2] Resensed poop is too far from sensed poop.  dist:%f.", closest2[2]);
                  sleep(2)
                else:
                  lost_goal_poop = False

              if not resense_failed:
                go_to_scoop_poop_at(base, closest2[0], closest2[1], 0)
                visualize_base_ray()
                speak("Scooping")
                sleep(0.5);


                # entering stage 3 #
#                logerr("\n\n [stage 3] Getting poops.")
#                points = poop_perception.get_last().points
#                poops = [(p.x, p.y) for p in points if p.z != 25]
#                if len(poops) == 0:
#                    break
            
#                base_x = base.get_x_map()
#                base_y = base.get_y_map()
#                poops_w_dist = [(p[0], p[1],
#                                 dist_between(p[0], p[1], base_x, base_y))
#                                for p in poops]
#                def get_dist(poop_w_dist):
#                    return poop_w_dist[2]
#                poops_w_dist.sort(key=get_dist)
#                closest = poops_w_dist[0]
#                if closest3[2] > 0.10:
#                    loginfo("[stage 3] Closest poop is further than 10cm away.");
#                    continue 
#                x, y, yaw = calc_work_x_y_yaw(base_x, base_y, closest3[0],closest3[1])
#                visualize_poop(x,y,0.02,2,"/map","reprojected_scoop")
 
                loginfo("[resense] Reached goal. Starting scoop.");
                scooper.scoop()
                head.lookNear()
            else:
              speak("Can't find the poop anymore.")
              logerr("Planning to poop failed.")
            sleep(5)
            #head.lookFar()
            #sleep(10)
    elif mode == 'start_scoop':
        scooper.start()
    elif mode == 'floor_scoop':
        scooper.floor()
    elif mode == 'look_near':
        head.lookNear()
    elif mode == 'look_far':
        head.lookFar()
    elif mode == 'hello':
        speak("Hello! It's time for me to scoop some poop!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
