from __future__ import division
from etfilter import *
import numpy as np
np.set_printoptions(suppress=True)

class Asset:

    def __init__(self, my_id, num_ownship_states, world_dim, x0, P0, predict_func, red_team=[]):
        self.my_id = my_id

        num_states = x0.size
        num_assets = int( num_states / num_ownship_states )
        
        # Initialize common filters between other assets
        self.common_filters = {}
        for i in range(num_assets):
            if (i != my_id) and (i not in red_team):
                self.common_filters[i] = ETFilter(my_id, num_ownship_states, world_dim, x0, P0, predict_func)

        # Initialize asset's primary filter
        self.main_filter = ETFilter_Main(my_id, num_ownship_states, world_dim, x0, P0, predict_func, self.common_filters)
    
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
            self.common_filters[asset_id].predict(u, Q)

    def correct(self):
        self.main_filter.correct()
        for asset_id in self.common_filters.keys():
            self.common_filters[asset_id].correct()

    def print_filters(self, main_only=False, mean_only=False):
        x_hat = np.matrix.round( deepcopy(self.main_filter.x_hat), 2)
        P = np.matrix.round( deepcopy(self.main_filter.P), 2)
        print(str(self.my_id)+"'s Main Filter")
        print(x_hat)
        if not mean_only:
            print(2*np.matrix.round(np.sqrt(P),2))

        if not main_only:
            print("----------")
            for asset_id in self.common_filters.keys():
                print(str(self.my_id) + "_" + str(asset_id) + " common filter")
                x_hat = np.matrix.round( deepcopy(self.common_filters[asset_id].x_hat), 2)
                P = np.matrix.round( deepcopy(self.common_filters[asset_id].P), 2)
                print(x_hat)
                if not mean_only:
                    print(np.matrix.round( 2*np.sqrt(self.common_filters[asset_id].P), 2))
                print("----------")

    def _get_implicit_msg_equivalent(self, meas):
        if isinstance(meas, GPSx_Explicit):
            return GPSx_Implicit(meas.src_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSy_Explicit):
            return GPSy_Implicit(meas.src_id, meas.R, meas.et_delta)
        elif isinstance(meas, GPSyaw_Explicit):
            return GPSyaw_Implicit(meas.src_id, meas.R, meas.et_delta)
        elif isinstance(meas, LinRelx_Explicit):
            return LinRelx_Implicit(meas.src_id, meas.measured_asset, meas.R, meas.et_delta)
        elif isinstance(meas, LinRely_Explicit):
            return LinRely_Implicit(meas.src_id, meas.measured_asset, meas.R, meas.et_delta)
        else:
            raise NotImplementedError("Implicit equivalent not found: " + meas.__class__.__name__)

    