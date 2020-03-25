class Measurement:
    src_id = None
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
    def __init__(self, src_id, data, R):
        self.src_id = src_id
        self.data = data
        self.R = R
class GPSy_Explicit(Explicit):
    def __init__(self, src_id, data, R):
        self.src_id = src_id
        self.data = data
        self.R = R
class GPSz_Explicit(Explicit): # Same as Depth_Explicit
    def __init__(self, src_id, data, R):
        self.src_id = src_id
        self.data = data
        self.R = R
class GPSx_Implicit(Implicit):
    def __init__(self, src_id, R):
        self.src_id = src_id
        self.R = R
class GPSy_Implicit(Implicit):
    def __init__(self, src_id, R):
        self.src_id = src_id
        self.R = R
class GPSz_Implicit(Implicit): # Same as Depth_Implicit
    def __init__(self, src_id, R):
        self.src_id = src_id
        self.R = R

########## LinRel ##########
class LinRelx_Explicit(Explicit):
    def __init__(self, src_id, measured_asset, delta, R):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.data = delta
        self.R = R
class LinRely_Explicit(Explicit):
    def __init__(self, src_id, measured_asset, delta, R):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.data = delta
        self.R = R
class LinRelx_Implicit(Implicit):
    def __init__(self, src_id, measured_asset, R):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.R = R
class LinRely_Implicit(Implicit):
    def __init__(self, src_id, measured_asset, R):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.R = R