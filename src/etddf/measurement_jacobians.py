from __future__ import division
import numpy as np
from etddf.measurements import *

def get_measurement_jacobian(meas, x_hat, num_states, world_dim, num_ownship_states):
    C = np.zeros((1, num_states))
    src_id = meas.src_id
    if isinstance(meas, GPSx_Explicit) or isinstance(meas, GPSx_Implicit):
        C[0, src_id*num_ownship_states] = 1
    elif isinstance(meas, GPSy_Explicit) or isinstance(meas, GPSy_Implicit):
        C[0, src_id*num_ownship_states + 1] = 1
    elif isinstance(meas, GPSz_Explicit) or isinstance(meas, GPSz_Implicit):
        C[0, src_id*num_ownship_states + 2] = 1
    elif isinstance(meas, GPSyaw_Explicit) or isinstance(meas, GPSyaw_Implicit):
        if world_dim == 2:
            C[0, src_id*num_ownship_states + 2] = 1
        else: # world dim 3
            C[0, src_id*num_ownship_states + 3] = 1
    elif isinstance(meas, GPSx_Neighbor_Explicit) or isinstance(meas, GPSx_Neighbor_Implicit):
        C[0, meas.neighbor_id*num_ownship_states] = 1
    elif isinstance(meas, GPSy_Neighbor_Explicit) or isinstance(meas, GPSy_Neighbor_Implicit):
        C[0, meas.neighbor_id*num_ownship_states+1] = 1
    elif isinstance(meas, GPSz_Neighbor_Explicit) or isinstance(meas, GPSz_Neighbor_Implicit):
        C[0, meas.neighbor_id*num_ownship_states+2] = 1
    elif isinstance(meas, GPSyaw_Neighbor_Explicit) or isinstance(meas, GPSyaw_Neighbor_Implicit):
        if world_dim == 2:
            C[0, meas.neighbor_id*num_ownship_states + 2] = 1
        else: # world dim 3
            C[0, meas.neighbor_id*num_ownship_states + 3] = 1
    elif isinstance(meas, Azimuth_Explicit) or isinstance(meas, Azimuth_Implicit):
        meas_id = meas.measured_asset_id
        src_x = x_hat[src_id*num_ownship_states,0]
        src_y = x_hat[src_id*num_ownship_states+1,0]
        other_x = x_hat[meas_id*num_ownship_states,0]
        other_y = x_hat[meas_id*num_ownship_states+1,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y

        # Protect division by zero
        diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
        diff_y = diff_y if abs(diff_y) > 0.01 else 0.01

        # Azimuth jacobians
        C[0, src_id*num_ownship_states] = diff_y / ( diff_x**2 + diff_y**2 )
        C[0, meas_id*num_ownship_states] = -diff_y / ( diff_x**2 + diff_y**2 )
        C[0, src_id*num_ownship_states+1] = -diff_x / ( diff_x**2 + diff_y**2 )
        C[0, meas_id*num_ownship_states+1] = diff_x / ( diff_x**2 + diff_y**2 )
        # d_az/d_theta is in a different index depending on 2D or 3D
        if world_dim == 2:
            C[0, src_id*num_ownship_states + 2] = -1
        else: # 3D World
            C[0, src_id*num_ownship_states + 3] = -1
    elif isinstance(meas, AzimuthGlobal_Explicit) or isinstance(meas, AzimuthGlobal_Implicit):
        src_x = x_hat[src_id*num_ownship_states,0]
        src_y = x_hat[src_id*num_ownship_states+1,0]
        other_x = meas.global_pose[0,0]
        other_y = meas.global_pose[1,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y

        # Protect division by zero
        diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
        diff_y = diff_y if abs(diff_y) > 0.01 else 0.01

        # Azimuth jacobians
        C[0, src_id*num_ownship_states] = diff_y / ( diff_x**2 + diff_y**2 )
        C[0, src_id*num_ownship_states+1] = -diff_x / ( diff_x**2 + diff_y**2 )
        # d_az/d_theta is in a different index depending on 2D or 3D
        if world_dim == 2:
            C[0, src_id*num_ownship_states + 2] = -1
        else: # 3D World
            C[0, src_id*num_ownship_states + 3] = -1
    elif isinstance(meas, AzimuthFromGlobal_Explicit) or isinstance(meas, AzimuthFromGlobal_Implicit):
        meas_id = meas.measured_asset_id
        src_x = meas.global_pose[0,0]
        src_y = meas.global_pose[1,0]

        other_x = x_hat[meas_id*num_ownship_states,0]
        other_y = x_hat[meas_id*num_ownship_states+1,0]
        
        diff_x = other_x - src_x
        diff_y = other_y - src_y

        # Protect division by zero
        diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
        diff_y = diff_y if abs(diff_y) > 0.01 else 0.01

        # Azimuth jacobians
        C[0, meas_id*num_ownship_states] = -diff_y / ( diff_x**2 + diff_y**2 )
        C[0, meas_id*num_ownship_states+1] = diff_x / ( diff_x**2 + diff_y**2 )

    elif isinstance(meas, Range_Explicit) or isinstance(meas, Range_Implicit):
        meas_id = meas.measured_asset_id
        src_x = x_hat[src_id*num_ownship_states,0]
        src_y = x_hat[src_id*num_ownship_states+1,0]
        other_x = x_hat[meas_id*num_ownship_states,0]
        other_y = x_hat[meas_id*num_ownship_states+1,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        
        if world_dim == 2:
            r = np.sqrt( diff_x**2 + diff_y**2 )
            r = r if r > 0.01 else 0.01 # Division by zero protection
            C[0, src_id*num_ownship_states] = -diff_x / r
            C[0, meas_id*num_ownship_states] = diff_x / r
            C[0, src_id*num_ownship_states+1] = -diff_y / r
            C[0, meas_id*num_ownship_states+1] = diff_y / r
        else: # World Dim 3D
            src_z = x_hat[src_id*num_ownship_states+2,0]
            other_z = x_hat[meas_id*num_ownship_states+2,0]
            diff_z = other_z - src_z
            r = np.sqrt( diff_x**2 + diff_y**2 + diff_z**2 )
            r = r if r > 0.01 else 0.01 # Division by zero protection
            C[0, src_id*num_ownship_states] = -diff_x / r
            C[0, meas_id*num_ownship_states] = diff_x / r
            C[0, src_id*num_ownship_states+1] = -diff_y / r
            C[0, meas_id*num_ownship_states+1] = diff_y / r
            C[0, src_id*num_ownship_states+2] = -diff_z / r
            C[0, meas_id*num_ownship_states+2] = diff_z / r

    elif isinstance(meas, RangeGlobal_Explicit) or isinstance(meas, RangeGlobal_Implicit):
        src_x = x_hat[src_id*num_ownship_states,0]
        src_y = x_hat[src_id*num_ownship_states+1,0]
        other_x = meas.global_pose[0,0]
        other_y = meas.global_pose[1,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        
        if world_dim == 2:
            r = np.sqrt( diff_x**2 + diff_y**2 )
            r = r if r > 0.01 else 0.01 # Division by zero protection
            C[0, src_id*num_ownship_states] = -diff_x / r
            C[0, src_id*num_ownship_states+1] = -diff_y / r
        else: # World Dim 3D
            src_z = x_hat[src_id*num_ownship_states+2,0]
            other_z = meas.global_pose[2,0]
            diff_z = other_z - src_z
            r = np.sqrt( diff_x**2 + diff_y**2 + diff_z**2 )
            r = r if r > 0.01 else 0.01 # Division by zero protection
            C[0, src_id*num_ownship_states] = -diff_x / r
            C[0, src_id*num_ownship_states+1] = -diff_y / r
            C[0, src_id*num_ownship_states+2] = -diff_z / r

    elif isinstance(meas, RangeFromGlobal_Explicit) or isinstance(meas, RangeFromGlobal_Implicit):
        meas_id = meas.measured_asset_id
        src_x = meas.global_pose[0,0]
        src_y = meas.global_pose[1,0]

        other_x = x_hat[meas_id*num_ownship_states,0]
        other_y = x_hat[meas_id*num_ownship_states+1,0]
        
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        
        if world_dim == 2:
            r = np.sqrt( diff_x**2 + diff_y**2 )
            r = r if r > 0.01 else 0.01 # Division by zero protection
            C[0, meas_id*num_ownship_states] = diff_x / r
            C[0, meas_id*num_ownship_states+1] = diff_y / r
        else: # World Dim 3D
            src_z = meas.global_pose[2,0]
            other_z = x_hat[meas_id*num_ownship_states+2,0]
            
            diff_z = other_z - src_z
            r = np.sqrt( diff_x**2 + diff_y**2 + diff_z**2 )
            r = r if r > 0.01 else 0.01 # Division by zero protection
            C[0, meas_id*num_ownship_states] = diff_x / r
            C[0, meas_id*num_ownship_states+1] = diff_y / r
            C[0, meas_id*num_ownship_states+2] = diff_z / r

    elif isinstance(meas, Elevation_Explicit) or isinstance(meas, Elevation_Implicit):
        meas_id = meas.measured_asset_id
        src_x = x_hat[meas.src_id*num_ownship_states,0]
        src_y = x_hat[meas.src_id*num_ownship_states+1,0]
        src_z = x_hat[meas.src_id*num_ownship_states+2,0]
        other_x = x_hat[meas_id*num_ownship_states,0]
        other_y = x_hat[meas_id*num_ownship_states+1,0]
        other_z = x_hat[meas_id*num_ownship_states+2,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        diff_z = other_z - src_z

        all_diff = diff_x**2 + diff_y**2 + diff_z**2
        all_diff = all_diff if all_diff > 0.01 else 0.01 # Division by zero protection
        
        ## Own asset part of jacobian
        # d_el / dx_src
        C[0, src_id*num_ownship_states] = (diff_z*diff_x) / (np.sqrt(1-(diff_z**2)/all_diff) * np.power(all_diff, 3/2) )
        # d_el / dy_src
        C[0, src_id*num_ownship_states+1] = (diff_z*diff_y) / (np.sqrt(1-(diff_z**2)/all_diff) * np.power(all_diff, 3/2) )
        # d_el / dz_src
        C[0, src_id*num_ownship_states+2] = - np.sqrt(diff_x**2 + diff_y**2) / all_diff

        ## Other Asset part of jacobian
        # d_el / dx_other
        C[0, meas_id*num_ownship_states] = -C[0, src_id*num_ownship_states]
        # d_el / dy_other
        C[0, meas_id*num_ownship_states+1] = -C[0, src_id*num_ownship_states]
        # d_el / dz_other
        C[0, meas_id*num_ownship_states+2] = -C[0, src_id*num_ownship_states+2]
    elif isinstance(meas, ElevationGlobal_Explicit) or isinstance(meas, ElevationGlobal_Implicit):
        src_x = x_hat[meas.src_id*num_ownship_states,0]
        src_y = x_hat[meas.src_id*num_ownship_states+1,0]
        src_z = x_hat[meas.src_id*num_ownship_states+2,0]
        other_x = meas.global_pose[0,0]
        other_y = meas.global_pose[1,0]
        other_z = meas.global_pose[2,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        diff_z = other_z - src_z

        all_diff = diff_x**2 + diff_y**2 + diff_z**2
        
        ## Own asset
        # d_el / dx_src
        C[0, src_id*num_ownship_states] = (diff_z*diff_x) / (np.sqrt(1-(diff_z**2)/all_diff) * np.power(all_diff, 3/2) )
        # d_el / dy_src
        C[0, src_id*num_ownship_states+1] = (diff_z*diff_y) / (np.sqrt(1-(diff_z**2)/all_diff) * np.power(all_diff, 3/2) )
        # d_el / dz_src
        C[0, src_id*num_ownship_states+2] = - np.sqrt(diff_x**2 + diff_y**2) / all_diff
    elif isinstance(meas, ElevationFromGlobal_Explicit) or isinstance(meas, ElevationFromGlobal_Implicit):
        meas_id = meas.measured_asset_id
        src_x = meas.global_pose[0,0]
        src_y = meas.global_pose[1,0]
        src_z = meas.global_pose[2,0]
        other_x = x_hat[meas_id*num_ownship_states,0]
        other_y = x_hat[meas_id*num_ownship_states+1,0]
        other_z = x_hat[meas_id*num_ownship_states+2,0]
        
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        diff_z = other_z - src_z

        all_diff = diff_x**2 + diff_y**2 + diff_z**2
        
        ## Own asset
        # d_el / dx_src
        C[0, meas_id*num_ownship_states] = -(diff_z*diff_x) / (np.sqrt(1-(diff_z**2)/all_diff) * np.power(all_diff, 3/2) )
        # d_el / dy_src
        C[0, meas_id*num_ownship_states+1] = -(diff_z*diff_y) / (np.sqrt(1-(diff_z**2)/all_diff) * np.power(all_diff, 3/2) )
        # d_el / dz_src
        C[0, meas_id*num_ownship_states+2] = np.sqrt(diff_x**2 + diff_y**2) / all_diff
    elif isinstance(meas, LinRelx_Explicit) or isinstance(meas, LinRelx_Implicit):
        src_id = meas.src_id
        meas_id = meas.measured_asset_id
        C[0,src_id*num_ownship_states] = -1
        C[0,meas_id*num_ownship_states] = 1
    elif isinstance(meas, LinRely_Explicit) or isinstance(meas, LinRely_Implicit):
        src_id = meas.src_id
        meas_id = meas.measured_asset_id
        C[0,src_id*num_ownship_states+1] = -1
        C[0,meas_id*num_ownship_states+1] = 1
    elif isinstance(meas, LinRelz_Explicit) or isinstance(meas, LinRelz_Implicit):
        src_id = meas.src_id
        meas_id = meas.measured_asset_id
        C[0,src_id*num_ownship_states+2] = -1
        C[0,meas_id*num_ownship_states+2] = 1
    elif isinstance(meas, Velocityx_Explicit) or isinstance(meas, Velocityx_Implicit):
        src_id = meas.src_id
        num_base_states = int(num_ownship_states / 2)
        C[0,src_id*num_ownship_states + num_base_states] = 1
    elif isinstance(meas, Velocityy_Explicit) or isinstance(meas, Velocityy_Implicit):
        src_id = meas.src_id
        num_base_states = int(num_ownship_states / 2)
        C[0,src_id*num_ownship_states + num_base_states + 1] = 1
    elif isinstance(meas, Velocityz_Explicit) or isinstance(meas, Velocityz_Implicit):
        src_id = meas.src_id
        num_base_states = int(num_ownship_states / 2)
        C[0,src_id*num_ownship_states + num_base_states + 2] = 1
    else:
        raise NotImplementedError("Measurment Jacobian not implemented for: " + meas.__class__.__name__)
    return C