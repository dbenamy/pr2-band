from collections import namedtuple


class MapFrameInt:
    """MapFrameInt acts more or less like an int, but represents a number in
    the map frame. Its purpose is to try to prevent mixing coordinate frames.
    
    """
    def __init__(self, val=None):
        self.val = val
        
    def __add__(self, other):
        """You can add a MapFrameInt to an int and get an MapFrameInt."""
        if not issubclass(other.__class__, int):
            raise TypeError("Invalid computation with non-MapFrameInt")
        return MapFrameInt(self.val + other)
    
    def __radd__(self, other):
        return self + other
    
    def __sub__(self, other):
        """MapFrameInt - MapFrameInt -> int
        MapFrameInt - int -> MapFrameInt
        
        """
        if issubclass(other.__class__, MapFrameInt):
            return self.val - other.val
        if issubclass(other.__class__, int):
            return MapFrameInt(self.val - other)
        else:
            raise TypeError("Invalid computation with non-MapFrameInt")
        
    def __repr__(self):
        return 'MapFrameType(%s)' % self.val


class MapPosition:
    """A position in the map frame."""
    def __init__(self, x, y, z):
        self.x = MapFrameInt(x)
        self.y = MapFrameInt(y)
        self.z = MapFrameInt(z)


class MapPose:
    """A pose in the map frame."""
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = MapFrameInt(x)
        self.y = MapFrameInt(y)
        self.z = MapFrameInt(z)
        self.roll = MapFrameInt(roll)
        self.pitch = MapFrameInt(pitch)
        self.yaw = MapFrameInt(yaw)


# An Obj is a physical object in the world. We may need to add more fields.
Obj = namedtuple(type, pose)
