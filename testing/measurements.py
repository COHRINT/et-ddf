class Measurement:
    src_id = None
    data = None
    R = None
    et_delta = None

""" All Measurements are either explicit or implicit """
class Implicit(Measurement):
    pass

class Explicit(Measurement):
    pass

""" Specific Measurement Enumerations """

########## GPS ##########
class GPSx_Explicit(Explicit):
    def __init__(self, src_id, data, R, et_delta):
        self.src_id = src_id
        self.data = data
        self.R = R
        self.et_delta = et_delta
class GPSy_Explicit(Explicit):
    def __init__(self, src_id, data, R, et_delta):
        self.src_id = src_id
        self.data = data
        self.R = R
        self.et_delta = et_delta
class GPSyaw_Explicit(Explicit):
    def __init__(self, src_id, data, R, et_delta):
        self.src_id = src_id
        self.data = data
        self.R = R
        self.et_delta = et_delta
class GPSx_Implicit(Implicit):
    def __init__(self, src_id, R, et_delta):
        self.src_id = src_id
        self.R = R
        self.et_delta = et_delta
class GPSy_Implicit(Implicit):
    def __init__(self, src_id, R, et_delta):
        self.src_id = src_id
        self.R = R
        self.et_delta = et_delta
class GPSyaw_Implicit(Implicit):
    def __init__(self, src_id, R, et_delta):
        self.src_id = src_id
        self.R = R
        self.et_delta = et_delta

########## LinRel ##########
class LinRelx_Explicit(Explicit):
    def __init__(self, src_id, measured_asset, data, R, et_delta):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.data = data
        self.R = R
        self.et_delta = et_delta
class LinRely_Explicit(Explicit):
    def __init__(self, src_id, measured_asset, data, R, et_delta):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.data = data
        self.R = R
        self.et_delta = et_delta
class LinRelx_Implicit(Implicit):
    def __init__(self, src_id, measured_asset, R, et_delta):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.R = R
        self.et_delta = et_delta
class LinRely_Implicit(Implicit):
    def __init__(self, src_id, measured_asset, R, et_delta):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.R = R
        self.et_delta = et_delta

########## Debug GPS of Neighbors ##########
# These measurements are impossible in reality but useful in debugging
# They represent "src took a gps measurement of neighbor"
class GPSx_Neighbor_Explicit(Explicit):
    def __init__(self, src_id, neighbor_id, data, R, et_delta):
        self.src_id = src_id
        self.neighbor_id = neighbor_id
        self.data = data
        self.R = R
        self.et_delta = et_delta
class GPSy_Neighbor_Explicit(Explicit):
    def __init__(self, src_id, neighbor_id, data, R, et_delta):
        self.src_id = src_id
        self.neighbor_id = neighbor_id
        self.data = data
        self.R = R
        self.et_delta = et_delta
class GPSx_Neighbor_Implicit(Implicit):
    def __init__(self, src_id, neighbor_id, R, et_delta):
        self.src_id = src_id
        self.neighbor_id = neighbor_id
        self.R = R
        self.et_delta = et_delta
class GPSy_Neighbor_Implicit(Implicit):
    def __init__(self, src_id, neighbor_id, R, et_delta):
        self.src_id = src_id
        self.neighbor_id = neighbor_id
        self.R = R
        self.et_delta = et_delta