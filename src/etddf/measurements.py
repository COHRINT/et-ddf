class Measurement:
    src_id = None
    data = None
    R = None
    et_delta = None
    is_linear_meas=True
    is_angle_meas=False

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
class GPSz_Explicit(Explicit):
    def __init__(self, src_id, data, R, et_delta):
        self.src_id = src_id
        self.data = data
        self.R = R
        self.et_delta = et_delta
class GPSyaw_Explicit(Explicit):
    def __init__(self, src_id, data, R, et_delta):
        self.src_id = src_id
        self.data = data # radians
        self.R = R
        self.et_delta = et_delta
        self.is_angle_meas = True
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
class GPSz_Implicit(Implicit):
    def __init__(self, src_id, R, et_delta):
        self.src_id = src_id
        self.R = R
        self.et_delta = et_delta
class GPSyaw_Implicit(Implicit):
    def __init__(self, src_id, R, et_delta):
        self.src_id = src_id
        self.R = R
        self.et_delta = et_delta
        self.is_angle_meas = True

########## Azimuth ##########
class Azimuth_Explicit(Explicit): # Relative
    def __init__(self, src_id, measured_asset, data, R, et_delta):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.data = data # radians
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class AzimuthGlobal_Explicit(Explicit):
    def __init__(self, src_id, global_pos, data, R, et_delta):
        self.src_id = src_id
        self.global_pos = global_pos # numpy 2D or 3D array
        self.data = data # radians
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class Azimuth_Implicit(Implicit): # Relative
    def __init__(self, src_id, measured_asset, R, et_delta):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class AzimuthGlobal_Implicit(Implicit):
    def __init__(self, src_id, global_pos, R, et_delta):
        self.src_id = src_id
        self.global_pos = global_pos # 2D or 3D array
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

########## Elevation ##########
class Elevation_Explicit(Explicit): # Relative
    def __init__(self, src_id, measured_asset, data, R, et_delta):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.data = data # radians
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class ElevationGlobal_Explicit(Explicit):
    def __init__(self, src_id, global_pos, data, R, et_delta):
        self.src_id = src_id
        self.global_pos = global_pos # 3D numpy array
        self.data = data # radians
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True
class Elevation_Implicit(Implicit): # Relative
    def __init__(self, src_id, measured_asset, R, et_delta):
        self.src_id = src_id
        self.measured_asset = measured_asset
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class ElevationGlobal_Implicit(Implicit):
    def __init__(self, src_id, global_pos, R, et_delta):
        self.src_id = src_id
        self.global_pos = global_pos # 3D array
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

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
# They represent "src took a gps measurement of neighbor" (as in I went to my neighbor's exact location and took a gps measurement for them)
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
class GPSz_Neighbor_Explicit(Explicit):
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
        self.data = data # radians
        self.R = R
        self.et_delta = et_delta
        self.is_angle_meas = True
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
class GPSz_Neighbor_Implicit(Implicit):
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
        self.is_angle_meas = True

""" Adding a new measurement Steps

Add it above & its implicit counterpart
Is it an angle? Add self.is_angle_meas = True to its constructor
If it has a nonlinear measurement function add it to _get_nonlinear_expected_meas()
Add its jacobian in etfilter to _get_measurement_jacobian()
Add its implicit conversion to asset.py
"""