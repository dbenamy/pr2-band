def plan_examine_surface(base_pose, surface_corners):
    """Returns a base pose which is close to the surface so the robot can best
    see what's on it. Only considers x and y.
    
    """
    pass


def plan_pickups(base_pose, pickup_objs):
    """Returns a list of places to go and objects to pick up at each place, in
    order of distance from current position. Something like:
    [
        (base pose, [
            (obj type, pose),
            ...
        ]),
        ...
    ]
    
    """
    pass
