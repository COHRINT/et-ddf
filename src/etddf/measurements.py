class Measurement:
    src_id = None
    measured_id = None
    data = None
    R = None
    et_delta = None
    is_linear_meas=True
    is_angle_meas=False

    def _print(self):
        """ Prints out member variables of a Measurement """
        print(self.__class__.__name__ + " : " + str(self.__dict__))

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
    def __init__(self, src_id, measured_asset_id, data, R, et_delta):
        self.src_id = src_id
        self.measured_asset_id = measured_asset_id
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

class AzimuthFromGlobal_Explicit(Explicit):
    def __init__(self, measured_asset_id, global_pose, data, R, et_delta):
        self.measured_asset_id = measured_asset_id
        self.global_pose = global_pose # 3D [x,y,z,yaw] or 2D [x,y,yaw]
        self.data = data # radians
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class Azimuth_Implicit(Implicit): # Relative
    def __init__(self, src_id, measured_asset_id, R, et_delta):
        self.src_id = src_id
        self.measured_asset_id = measured_asset_id
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class AzimuthGlobal_Implicit(Implicit):
    def __init__(self, src_id, global_pose, R, et_delta):
        self.src_id = src_id
        self.global_pose = global_pose # 3D [x,y,z,yaw] or 2D [x,y,yaw] (yaw isn't used)
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class AzimuthFromGlobal_Implicit(Implicit):
    def __init__(self, measured_asset_id, global_pose, R, et_delta):
        self.measured_asset_id = measured_asset_id
        self.global_pose = global_pose # 3D [x,y,z,yaw] or 2D [x,y,yaw]
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

########## Elevation ##########
class Elevation_Explicit(Explicit): # Relative
    def __init__(self, src_id, measured_asset_id, data, R, et_delta):
        self.src_id = src_id
        self.measured_asset_id = measured_asset_id
        self.data = data # radians
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class ElevationGlobal_Explicit(Explicit):
    def __init__(self, src_id, global_pose, data, R, et_delta):
        self.src_id = src_id
        self.global_pose = global_pose # 3D [x,y,z,yaw]
        self.data = data # radians
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class ElevationFromGlobal_Explicit(Explicit):
    def __init__(self, measured_asset_id, global_pose, data, R, et_delta):
        self.measured_asset_id = measured_asset_id
        self.global_pose = global_pose # np array [x,y,z,yaw]
        self.data = data # radians
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class Elevation_Implicit(Implicit): # Relative
    def __init__(self, src_id, measured_asset_id, R, et_delta):
        self.src_id = src_id
        self.measured_asset_id = measured_asset_id
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class ElevationGlobal_Implicit(Implicit):
    def __init__(self, src_id, global_pose, R, et_delta):
        self.src_id = src_id
        self.global_pose = global_pose # 3D [x,y,z,yaw]
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

class ElevationFromGlobal_Implicit(Implicit):
    def __init__(self, measured_asset_id, global_pose, R, et_delta):
        self.measured_asset_id = measured_asset_id
        self.global_pose = global_pose # 3D [x,y,z,yaw]
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False
        self.is_angle_meas = True

########## Range ##########
class Range_Explicit(Explicit): # Relative
    def __init__(self, src_id, measured_asset_id, data, R, et_delta):
        self.src_id = src_id
        self.measured_asset_id = measured_asset_id
        self.data = data
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

class RangeGlobal_Explicit(Explicit):
    def __init__(self, src_id, global_pose, data, R, et_delta):
        self.src_id = src_id
        self.global_pose = global_pose # 3D [x,y,z,yaw] or 2D [x,y,yaw] (yaw isn't used)
        self.data = data
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

class RangeFromGlobal_Explicit(Explicit):
    def __init__(self, measured_asset_id, global_pose, data, R, et_delta):
        self.measured_asset_id = measured_asset_id
        self.global_pose = global_pose # 3D [x,y,z,yaw] or 2D [x,y,yaw] (yaw isn't used)
        self.data = data
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

class Range_Implicit(Implicit): # Relative
    def __init__(self, src_id, measured_asset_id, R, et_delta):
        self.src_id = src_id
        self.measured_asset_id = measured_asset_id
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

class RangeGlobal_Implicit(Implicit):
    def __init__(self, src_id, global_pose, R, et_delta):
        self.src_id = src_id
        self.global_pose = global_pose # 3D [x,y,z,yaw] or 2D [x,y,yaw] (yaw isn't used)
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

class RangeFromGlobal_Implicit(Implicit):
    def __init__(self, src_id, global_pose, data, R, et_delta):
        self.global_pose = global_pose # 3D [x,y,z,yaw] or 2D [x,y,yaw] (yaw isn't used)
        self.data = data
        self.R = R
        self.et_delta = et_delta
        self.is_linear_meas = False

########## Linear Relation ##########
class LinRelx_Explicit(Explicit):
    def __init__(self, src_id, measured_asset_id, data, R, et_delta):
        self.src_id = src_id
        self.measured_asset_id = measured_asset_id
        self.data = data
        self.R = R
        self.et_delta = et_delta

class LinRely_Explicit(Explicit):
    def __init__(self, src_id, measured_asset_id, data, R, et_delta):
        self.src_id = src_id
        self.measured_asset_id = measured_asset_id
        self.data = data
        self.R = R
        self.et_delta = et_delta

class LinRelz_Explicit(Explicit):
    def __init__(self, src_id, measured_asset_id, data, R, et_delta):
        self.src_id = src_id
        self.measured_asset_id = measured_asset_id
        self.data = data
        self.R = R
        self.et_delta = et_delta

class LinRelx_Implicit(Implicit):
    def __init__(self, src_id, measured_asset_id, R, et_delta):
        self.src_id = src_id
        self.measured_asset_id = measured_asset_id
        self.R = R
        self.et_delta = et_delta

class LinRely_Implicit(Implicit):
    def __init__(self, src_id, measured_asset_id, R, et_delta):
        self.src_id = src_id
        self.measured_asset_id = measured_asset_id
        self.R = R
        self.et_delta = et_delta

class LinRelz_Implicit(Implicit):
    def __init__(self, src_id, measured_asset_id, R, et_delta):
        self.src_id = src_id
        self.measured_asset_id = measured_asset_id
        self.R = R
        self.et_delta = et_delta

########## Velocity ##########
class Velocityx_Explicit(Explicit):
    def __init__(self, src_id, data, R, et_delta):
        self.src_id = src_id
        self.data = data
        self.R = R
        self.et_delta = et_delta
class Velocityy_Explicit(Explicit):
    def __init__(self, src_id, data, R, et_delta):
        self.src_id = src_id
        self.data = data
        self.R = R
        self.et_delta = et_delta
class Velocityz_Explicit(Explicit):
    def __init__(self, src_id, data, R, et_delta):
        self.src_id = src_id
        self.data = data
        self.R = R
        self.et_delta = et_delta
class Velocityx_Implicit(Implicit):
    def __init__(self, src_id, R, et_delta):
        self.src_id = src_id
        self.R = R
        self.et_delta = et_delta
class Velocityy_Implicit(Implicit):
    def __init__(self, src_id, R, et_delta):
        self.src_id = src_id
        self.R = R
        self.et_delta = et_delta
class Velocityz_Implicit(Implicit):
    def __init__(self, src_id, R, et_delta):
        self.src_id = src_id
        self.R = R
        self.et_delta = et_delta

########## Debug GPS of Neighbors ##########
# These measurements are impossible in reality but useful in debugging
# They represent "src took a gps measurement of neighbor" (e.g. I went to my neighbor's exact location and took a gps measurement for them)
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
Add its jacobian to get_measurement_jacobian() in measurement_jacobians.py
If it has a nonlinear measurement function, add it to get_nonlinear_expected_meas() in measurement_expected.py
Add its implicit conversion to asset.py
"""