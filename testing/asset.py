from __future__ import division
from etfilter import *
import numpy as np
np.set_printoptions(suppress=True)

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
                    sharing[asset_id] = self._get_implicit_msg_equivalent(meas)
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
            self.common_filters[asset_id].predict(np.zeros(u.shape), Q)

    def correct(self):
        self.main_filter.correct()
        for asset_id in self.common_filters.keys():
            self.common_filters[asset_id].correct()

    def print_filters(self, main_only=False, mean_only=False, cov_sd_form=True, round_dec=2):
        asset_ids = np.linspace(0, self.num_assets-1, self.num_assets).reshape(-1,1)
        num_ownship_states = int(self.num_states / self.num_assets)
        z = (np.zeros((self.num_assets, num_ownship_states)) + asset_ids).reshape(-1,1)

        x_hat = np.matrix.round( deepcopy(self.main_filter.x_hat), round_dec)
        P = deepcopy(self.main_filter.P)
        print(str(self.my_id)+"'s Main Filter")
        print(np.concatenate((z,x_hat),axis=1))
        if not mean_only:
            if cov_sd_form:
                print(np.matrix.round( 2*np.sqrt(P), round_dec))
            else:
                print(np.matrix.round(P, round_dec))

        if not main_only:
            print("----------")
            for asset_id in self.common_filters.keys():
                print(str(self.my_id) + "_" + str(asset_id) + " common filter")
                x_hat = np.matrix.round( deepcopy(self.common_filters[asset_id].x_hat), round_dec)
                P = deepcopy(self.common_filters[asset_id].P)
                print(x_hat)
                if not mean_only:
                    if cov_sd_form:
                        print(np.matrix.round( 2*np.sqrt(P), round_dec))
                    else:
                        print(np.matrix.round(P, round_dec))
                print("----------")

    def _get_implicit_msg_equivalent(self, meas):
        if isinstance(meas, GPSx_Explicit):
            return GPSx_Implicit(meas.src_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSy_Explicit):
            return GPSy_Implicit(meas.src_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSyaw_Explicit):
            return GPSyaw_Implicit(meas.src_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSx_Neighbor_Explicit):
            return GPSx_Neighbor_Implicit(meas.src_id, meas.neighbor_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSy_Neighbor_Explicit):
            return GPSy_Neighbor_Implicit(meas.src_id, meas.neighbor_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSyaw_Neighbor_Explicit):
            return GPSyaw_Neighbor_Implicit(meas.src_id, meas.neighbor_id, meas.R, meas.et_delta)
        elif isinstance(meas, LinRelx_Explicit):
            return LinRelx_Implicit(meas.src_id, meas.measured_asset, meas.R, meas.et_delta)
        elif isinstance(meas, LinRely_Explicit):
            return LinRely_Implicit(meas.src_id, meas.measured_asset, meas.R, meas.et_delta)
        elif isinstance(meas, Azimuth_Explicit):
            return Azimuth_Implicit(meas.src_id, meas.measured_asset, meas.R, meas.et_delta)
        elif isinstance(meas, AzimuthGlobal_Explicit):
            return AzimuthGlobal_Implicit(meas.src_id, meas.global_pos, meas.R, meas.et_delta)
        elif isinstance(meas, Range_Explicit):
            return Range_Implicit(meas.src_id, meas.measured_asset, meas.R, meas.et_delta)
        elif isinstance(meas, RangeGlobal_Explicit):
            return RangeGlobal_Implicit(meas.src_id, meas.global_pos, meas.R, meas.et_delta)
        else:
            raise NotImplementedError("Implicit equivalent not found: " + meas.__class__.__name__)

    