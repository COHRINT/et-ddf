from __future__ import division
from etddf.etfilter import *
from etddf.measurements import *
import numpy as np

class Asset:

    def __init__(self, my_id, num_ownship_states, world_dim, x0, P0, linear_dynamics, red_team=[]):
        self.my_id = my_id

        self.num_states = x0.size
        self.num_assets = int( self.num_states / num_ownship_states )
        
        # Initialize common filters between other assets
        self.common_filters = {}
        for i in range(self.num_assets):
            if (i != my_id) and (i not in red_team):
                self.common_filters[i] = ETFilter(my_id, num_ownship_states, world_dim, x0, P0, linear_dynamics)

        # Initialize asset's primary filter
        self.main_filter = ETFilter_Main(my_id, num_ownship_states, world_dim, x0, P0, linear_dynamics, self.common_filters)
    
    def receive_meas(self, meas, shareable=True):
        self.main_filter.add_meas(meas)

        # Determine if other assets should get an implicit or explicit version of this meas
        if shareable and len(self.common_filters) > 0:
            sharing = {}
            for asset_id in self.common_filters.keys():
                common_filter = self.common_filters[asset_id]
                if common_filter.check_implicit(meas):
                    sharing[asset_id] = self.get_implicit_msg_equivalent(meas)
                else: # share as explicit
                    sharing[asset_id] = meas
                    
                # Add the measurement to the common filter
                common_filter.add_meas(sharing[asset_id])
            return sharing
        else: # Not shareable
            return None

    def receive_shared_meas(self, meas):
        self.main_filter.add_meas(meas)
        self.common_filters[meas.src_id].add_meas(meas)

    def predict(self, u, Q):
        self.main_filter.predict(u, Q)
        for asset_id in self.common_filters.keys():
            # Other assets don't have access to our control input, so they assume constant velocity
            self.common_filters[asset_id].predict(np.zeros(u.shape), Q)

    def correct(self):
        self.main_filter.correct()
        for asset_id in self.common_filters.keys():
            self.common_filters[asset_id].correct()

    @staticmethod
    def get_implicit_msg_equivalent(meas):
        if isinstance(meas, GPSx_Explicit):
            return GPSx_Implicit(meas.src_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSy_Explicit):
            return GPSy_Implicit(meas.src_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSz_Explicit):
            return GPSz_Implicit(meas.src_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSyaw_Explicit):
            return GPSyaw_Implicit(meas.src_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSx_Neighbor_Explicit):
            return GPSx_Neighbor_Implicit(meas.src_id, meas.neighbor_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSy_Neighbor_Explicit):
            return GPSy_Neighbor_Implicit(meas.src_id, meas.neighbor_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSz_Neighbor_Explicit):
            return GPSz_Neighbor_Implicit(meas.src_id, meas.neighbor_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSyaw_Neighbor_Explicit):
            return GPSyaw_Neighbor_Implicit(meas.src_id, meas.neighbor_id, meas.R, meas.et_delta)
        elif isinstance(meas, Azimuth_Explicit):
            return Azimuth_Implicit(meas.src_id, meas.measured_asset_id, meas.R, meas.et_delta)
        elif isinstance(meas, AzimuthGlobal_Explicit):
            return AzimuthGlobal_Implicit(meas.src_id, meas.global_pos, meas.R, meas.et_delta)
        elif isinstance(meas, Elevation_Explicit):
            return Elevation_Implicit(meas.src_id, meas.measured_asset_id, meas.R, meas.et_delta)
        elif isinstance(meas, ElevationGlobal_Explicit):
            return ElevationGlobal_Implicit(meas.src_id, meas.global_pos, meas.R, meas.et_delta)
        elif isinstance(meas, Range_Explicit):
            return Range_Implicit(meas.src_id, meas.measured_asset_id, meas.R, meas.et_delta)
        elif isinstance(meas, RangeGlobal_Explicit):
            return RangeGlobal_Implicit(meas.src_id, meas.global_pos, meas.R, meas.et_delta)
        elif isinstance(meas, LinRelx_Explicit):
            return LinRelx_Implicit(meas.src_id, meas.measured_asset_id, meas.R, meas.et_delta)
        elif isinstance(meas, LinRely_Explicit):
            return LinRely_Implicit(meas.src_id, meas.measured_asset_id, meas.R, meas.et_delta)
        elif isinstance(meas, LinRelz_Explicit):
            return LinRelz_Implicit(meas.src_id, meas.measured_asset_id, meas.R, meas.et_delta)
        elif isinstance(meas, Velocityx_Explicit):
            return Velocityx_Implicit(meas.src_id, meas.R, meas.et_delta)
        elif isinstance(meas, Velocityy_Explicit):
            return Velocityy_Implicit(meas.src_id, meas.R, meas.et_delta)
        elif isinstance(meas, Velocityz_Explicit):
            return Velocityz_Implicit(meas.src_id, meas.R, meas.et_delta)
        else:
            raise NotImplementedError("Implicit equivalent not found: " + meas.__class__.__name__)

    