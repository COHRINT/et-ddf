from __future__ import division
import numpy as np
from etddf.measurements import *
from etddf.normalize_angle import *

def get_nonlinear_expected_meas(meas, x_hat, world_dim, num_ownship_states):
    if isinstance(meas, Azimuth_Explicit) or isinstance(meas, Azimuth_Implicit):
        if world_dim == 2:
            src_bearing = x_hat[meas.src_id*num_ownship_states + 2,0]
        else:
            src_bearing = x_hat[meas.src_id*num_ownship_states + 3,0]
        src_x = x_hat[meas.src_id*num_ownship_states,0]
        src_y = x_hat[meas.src_id*num_ownship_states+1,0]
        other_x = x_hat[meas.measured_asset_id*num_ownship_states,0]
        other_y = x_hat[meas.measured_asset_id*num_ownship_states+1,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y

        # Protect division by zero
        diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
        diff_y = diff_y if abs(diff_y) > 0.01 else 0.01
        expected_bearing = np.arctan2(diff_y, diff_x) - src_bearing
        return normalize_angle( expected_bearing )
    elif isinstance(meas, AzimuthGlobal_Explicit) or isinstance(meas, AzimuthGlobal_Implicit):
        if world_dim == 2:
            src_bearing = x_hat[meas.src_id*num_ownship_states + 2,0]
        else:
            src_bearing = x_hat[meas.src_id*num_ownship_states + 3,0]
        src_x = x_hat[meas.src_id*num_ownship_states,0]
        src_y = x_hat[meas.src_id*num_ownship_states+1,0]
        other_x = meas.global_pose[0,0]
        other_y = meas.global_pose[1,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y

        # Protect division by zero
        diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
        diff_y = diff_y if abs(diff_y) > 0.01 else 0.01
        expected_bearing = np.arctan2(diff_y, diff_x) - src_bearing
        return normalize_angle( expected_bearing )
    elif isinstance(meas, AzimuthFromGlobal_Explicit) or isinstance(meas, AzimuthFromGlobal_Implicit):
        meas_id = meas.measured_asset_id          
        if world_dim == 2:
            src_bearing = meas.global_pose[2,0]
        else:
            src_bearing = meas.global_pose[3,0]
        src_x = meas.global_pose[0,0]
        src_y = meas.global_pose[1,0]
        other_x = x_hat[meas_id*num_ownship_states,0]
        other_y = x_hat[meas_id*num_ownship_states+1,0]
        
        diff_x = other_x - src_x
        diff_y = other_y - src_y

        # Protect division by zero
        diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
        diff_y = diff_y if abs(diff_y) > 0.01 else 0.01
        expected_bearing = np.arctan2(diff_y, diff_x) - src_bearing
        return normalize_angle( expected_bearing )
    elif isinstance(meas, Range_Explicit) or isinstance(meas, Range_Implicit):
        src_x = x_hat[meas.src_id*num_ownship_states,0]
        src_y = x_hat[meas.src_id*num_ownship_states+1,0]
        other_x = x_hat[meas.measured_asset_id*num_ownship_states,0]
        other_y = x_hat[meas.measured_asset_id*num_ownship_states+1,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        if world_dim == 2:
            return np.sqrt( diff_x**2 + diff_y**2 )
        else: # 3D World
            src_z = x_hat[meas.src_id*num_ownship_states+2,0]
            other_z = x_hat[meas.measured_asset_id*num_ownship_states+2,0]
            diff_z = other_z - src_z
            return np.sqrt( diff_x**2 + diff_y**2 + diff_z**2 )
            
    elif isinstance(meas, RangeGlobal_Explicit) or isinstance(meas, RangeGlobal_Implicit):
        src_x = x_hat[meas.src_id*num_ownship_states,0]
        src_y = x_hat[meas.src_id*num_ownship_states+1,0]
        other_x = meas.global_pose[0,0]
        other_y = meas.global_pose[1,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        if world_dim == 2:
            return np.sqrt( diff_x**2 + diff_y**2 )
        else: # 3D World
            src_z = x_hat[meas.src_id*num_ownship_states+2,0]
            other_z = meas.global_pose[2,0]
            diff_z = other_z - src_z
            return np.sqrt( diff_x**2 + diff_y**2 + diff_z**2 )
    elif isinstance(meas, RangeFromGlobal_Explicit) or isinstance(meas, RangeFromGlobal_Implicit):
        meas_id = meas.measured_asset_id
        src_x = meas.global_pose[0,0]
        src_y = meas.global_pose[1,0]
        other_x = x_hat[meas_id*num_ownship_states,0]
        other_y = x_hat[meas_id*num_ownship_states+1,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        if world_dim == 2:
            return np.sqrt( diff_x**2 + diff_y**2 )
        else: # 3D World
            src_z = meas.global_pose[2,0]
            other_z = x_hat[meas_id*num_ownship_states+2,0]
            diff_z = other_z - src_z
            return np.sqrt( diff_x**2 + diff_y**2 + diff_z**2 )
    elif isinstance(meas, Elevation_Explicit) or isinstance(meas, Elevation_Implicit):
        src_x = x_hat[meas.src_id*num_ownship_states,0]
        src_y = x_hat[meas.src_id*num_ownship_states+1,0]
        src_z = x_hat[meas.src_id*num_ownship_states+2,0]
        other_x = x_hat[meas.measured_asset_id*num_ownship_states,0]
        other_y = x_hat[meas.measured_asset_id*num_ownship_states+1,0]
        other_z = x_hat[meas.measured_asset_id*num_ownship_states+2,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        diff_z = other_z - src_z

        # Protect division by zero
        diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
        diff_y = diff_y if abs(diff_y) > 0.01 else 0.01
        diff_z = diff_z if abs(diff_z) > 0.01 else 0.01

        expected_elevation = np.arcsin(diff_z / np.linalg.norm([diff_x, diff_y, diff_z]))
        return normalize_angle( expected_elevation )
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

        # Protect division by zero
        diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
        diff_y = diff_y if abs(diff_y) > 0.01 else 0.01
        diff_z = diff_z if abs(diff_z) > 0.01 else 0.01

        expected_elevation = np.arcsin(diff_z / np.linalg.norm([diff_x, diff_y, diff_z]))
        return normalize_angle( expected_elevation )
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

        # Protect division by zero
        diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
        diff_y = diff_y if abs(diff_y) > 0.01 else 0.01
        diff_z = diff_z if abs(diff_z) > 0.01 else 0.01

        expected_elevation = np.arcsin(diff_z / np.linalg.norm([diff_x, diff_y, diff_z]))
        return normalize_angle( expected_elevation )
    else:
        raise NotImplementedError("Nonlinear Measurement Innovation not implemented for: " + meas.__class__.__name__)