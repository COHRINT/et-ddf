class Measurement:
    data = None
    R = None

""" All Measurements are either explicit or implicit """
class Implicit(Measurement):
    pass

class Explicit(Measurement):
    pass

""" Specific Measurement Enumerations """

########## GPS ##########
class GPSx_Explicit(Explicit):
    def __init__(self, x, R):
        self.data = x
        self.R = R
class GPSy_Explicit(Explicit):
    def __init__(self, x, R):
        self.data = x
        self.R = R
class GPSz_Explicit(Explicit): # Same as Depth_Explicit
    def __init__(self, x, R):
        self.data = x
        self.R = R
class GPSx_Implicit(Implicit):
    def __init__(self, R):
        self.R = R # data is None b/c it's implicit!
class GPSy_Implicit(Implicit):
    def __init__(self, R):
        self.R = R
class GPSz_Implicit(Implicit): # Same as Depth_Implicit
    def __init__(self, R):
        self.R = R

########## LinRel ##########
class LinRelx_Explicit(Explicit):
    def __init__(self, delta, other_asset, R):
        self.data = delta
        self.other_asset = other_asset
        self.R = R
class LinRely_Explicit(Explicit):
    def __init__(self, delta, other_asset, R):
        self.data = delta
        self.other_asset = other_asset
        self.R = R
class LinRelz_Explicit(Explicit):
    def __init__(self, delta, other_asset, R):
        self.data = delta
        self.other_asset = other_asset
        self.R = R
class LinRelx_Implicit(Implicit):
    def __init__(self, other_asset, R):
        self.other_asset = other_asset
        self.R = R
class LinRely_Implicit(Implicit):
    def __init__(self, other_asset, R):
        self.other_asset = other_asset
        self.R = R
class LinRelz_Implicit(Implicit):
    def __init__(self, other_asset, R):
        self.other_asset = other_asset
        self.R = R