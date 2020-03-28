class Measurement:
    src_id = None
    data = None
    R = None
    et_delta = None
    is_linear_meas=True

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

########## Bearing ##########
class Azimuth_Explicit(Explicit): # Relative
    def __init__(self, src_id, measured_asset, data, R, et_delta):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.data = data
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

class AzimuthGlobal_Explicit(Explicit):
    def __init__(self, src_id, global_pos, data, R, et_delta):
        self.src_id = src_id
        self.global_pos = global_pos # numpy 2D or 3D array
        self.data = data
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

class Azimuth_Implicit(Implicit): # Relative
    def __init__(self, src_id, measured_asset, R, et_delta):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

class AzimuthGlobal_Implicit(Implicit):
    def __init__(self, src_id, global_pos, R, et_delta):
        self.src_id = src_id
        self.global_pos = global_pos # numpy 2D or 3D array
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

########## Range ##########
class Range_Explicit(Explicit): # Relative
    def __init__(self, src_id, measured_asset, data, R, et_delta):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.data = data
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

class RangeGlobal_Explicit(Explicit):
    def __init__(self, src_id, global_pos, data, R, et_delta):
        self.src_id = src_id
        self.global_pos = global_pos # numpy 2D or 3D array
        self.data = data
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

class Range_Implicit(Implicit): # Relative
    def __init__(self, src_id, measured_asset, R, et_delta):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

class RangeGlobal_Implicit(Implicit):
    def __init__(self, src_id, global_pos, R, et_delta):
        self.src_id = src_id
        self.global_pos = global_pos # numpy 2D or 3D array
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

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
class GPSyaw_Neighbor_Explicit(Explicit):
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
class GPSyaw_Neighbor_Implicit(Implicit):
    def __init__(self, src_id, neighbor_id, R, et_delta):
        self.src_id = src_id
        self.neighbor_id = neighbor_id
        self.R = R
        self.et_delta = et_delta

""" Adding a new measurement Steps

Add it above & its implicit counterpart
Is it an angle? Add it to _is_angle_meas() in etfilter
If it has a nonlinear measurement function add it to _get_nonlinear_expected_meas()
Add its jacobian in etfilter to _get_measurement_jacobian()
Add its implicit conversion to asset.py
"""