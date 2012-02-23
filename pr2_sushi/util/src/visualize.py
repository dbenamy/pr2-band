import roslib; roslib.load_manifest('util')


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