from math import sqrt


def dist_between(*args):
    """Call as dist_between(position1, position2) or
    dist_between(x1, y1, x2, y2).
    
    """
    if len(args) == 2:
        return dist_between_2(*args)
    elif len(args) == 4:
        return dist_between_4(*args)
    else:
        raise ValueError("Invalid number of arguments")


def dist_between_2(pos1, pos2):
    return dist_between_4(pos1.x, pos1.y, pos2.x, pos2.y)
    
def dist_between_4(x1, y1, x2, y2):
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)


def point_inside_polygon(point, poly):
    """Determine if a point is inside a given polygon.
    point is a tuple of x, y. poly is a list of (x, y) tuples.
    Eg: point_inside_polygon((1, 2), [(0, 0), (10, 0), (0, 10)]) -> True
    http://www.ariel.com.au/a/python-point-int-poly.html
    
    """
    x, y = point
    n = len(poly)
    inside = False
    p1x, p1y = poly[0]
    for i in range(n + 1):
        p2x, p2y = poly[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


def calc_work_x_y_yaw(base_x_map, base_y_map, target_x_map, target_y_map, working_dist):
    """Given the base position, a target object position, and the distance to
    be from the target to work on it, computes the nearest position and yaw for
    the base which will put it at that working distance from the target.
    Assumes there are no obstacles.
    
    """
    # Variable name convention is: b = base, t = target, w = working position.
    # For example, wt_dx = the dx between the working position and target
    # position.
    b_x = base_x_map
    b_y = base_y_map
    t_x = target_x_map
    t_y = target_y_map
    wt_dist = working_dist
    
    # Find working position
    bt_dx = t_x - b_x
    bt_dy = t_y - b_y
    bt_dist = sqrt(bt_dx**2 + bt_dy**2)
    fraction = wt_dist / bt_dist
    wt_dx = fraction * bt_dx
    wt_dy = fraction * bt_dy
    w_x = t_x - wt_dx
    w_y = t_y - wt_dy
    
    # Find working yaw
    w_yaw = atan2(bt_dy, bt_dx)
    
    return w_x, w_y, w_yaw


def calc_work_pose(base, obj, working_dist):
    """Same as calc_work_x_y_yaw but takes MapPositions or MapPoses and returns
    a MapPose.
    
    """
#    if not issubtype(base.__class__, MapVal):
#        raise TypeError()
#    if not issubtype(obj.__class__, MapVal):
#        raise TypeError()
    x, y, yaw = calc_work_x_y_yaw(base.x.val, base.y.val, target.x.val,
                                  target.y.val, working_dist)
    return MapPose(x, y, 0, 0, 0, yaw)

    
def calc_point_along_line(x1, y1, x2, y2, dist):
    """Returns a point on the line through point1 and point2 which is dist
    away from point 2 in the direction away from point 1. For example,
    calc_point_along_line(0, 0, 10, 10, 1) will return approximately
    (10.707 10.707).
    
    """
    one_to_two_vector = (x2 - x1, y2 - y1)
    magnitude = sqrt(one_to_two_vector[0] ** 2 + one_to_two_vector[1] ** 2)
    unit_vector = (one_to_two_vector[0] / magnitude, one_to_two_vector[1] / magnitude)
    dist_vector = (dist * unit_vector[0], dist * unit_vector[1])
    return (x2 + dist_vector[0], y2 + dist_vector[1])


def pose_obj_to_map(obj, pose):
    """Converts pose which is in the object's frame to the map frame."""
    raise NotImplemented()
    """
    def current_position(self):
        self.transformer.waitForTransform('map', 'base_footprint', Time.now())
        base_in_base = PointStamped()
        base_in_base.header.frame_id = 'base_footprint'
        base_in_base.header.stamp = Time.now()
        base_in_map = transformer.transformPoint('map', base_in_base)
        return (current_point.point.x, current_point.point.y)
    """