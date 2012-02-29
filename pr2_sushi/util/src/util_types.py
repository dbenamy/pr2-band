class MapFrameInt:
    """MapFrameInt acts more or less like an int, but represents a number in
    the map frame and can only be used automatically with other MapFrameInts.
    
    """
    def __init__(self, val=None):
        self.val = val
        
    def __add__(self, other):
        self._check_type(other)
        return MapFrameInt(self.val + other.val)
        
    def __sub__(self, other):
        self._check_type(other)
        return MapFrameInt(self.val - other.val)
        
    def __mul__(self, other):
        self._check_type(other)
        return MapFrameInt(self.val * other.val)
        
    def __div__(self, other):
        self._check_type(other)
        return MapFrameInt(self.val / other.val)
        
    def __mod__(self, other):
        self._check_type(other)
        return MapFrameInt(self.val % other.val)
    
    def _check_type(self, other):
        if not issubclass(other.__class__, MapFrameInt):
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
