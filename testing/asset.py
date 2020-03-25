from __future__ import division
from etfilter import *
import numpy as np

class Asset:

    def __init__(self, my_id, num_states_per_asset, x0, P0, A, B, et_delta, red_team=[]):
        self.my_id = my_id

        num_states = x0.size
        if (num_states % num_states_per_asset) != 0:
            raise Exception("Dimensionality of state vector does not align with the number states per asset.")
        num_assets = int( num_states / num_states_per_asset )
        
        # Initialize common filters between other assets
        self.common_filters = {}
        for i in range(num_assets):
            if (i != my_id) and (i not in red_team):
                self.common_filters[i] = ETFilter(my_id, num_states_per_asset, x0, P0, A, B, et_delta)

        # Initialize asset's primary filter
        self.main_filter = ETFilter_Main(my_id, num_states_per_asset, x0, P0, A, B, et_delta, self.common_filters)
    
    def receive_meas(self, meas, shareable=True):
        self.main_filter.add_meas(self.my_id, meas)

        # Determine if other assets should get an implicit or explicit version of this meas
        if shareable and len(self.common_filters) > 0:
            sharing = {}
            for asset_id in self.common_filters.keys():
                common_filter = self.common_filters[asset_id]
                if common_filter.check_implicit(self.my_id, meas):
                    sharing[asset_id] = self._get_implicit_msg_equivalent(meas)
                else: # share as explicit
                    sharing[asset_id] = meas
                    
                # Add the measurement to the common filter
                common_filter.add_meas(self.my_id, sharing[asset_id])
            return sharing
        else: # Not shareable
            return None

    def receive_shared_meas(self, asset_id, meas):
        self.main_filter.add_meas(asset_id, meas)
        self.common_filters[asset_id].add_meas(asset_id, meas)    

    def predict(self, u, Q):
        self.main_filter.predict(u, Q)
        for asset_id in self.common_filters.keys():
            self.common_filters[asset_id].predict(u, Q)

    def correct(self):
        for asset_id in self.common_filters.keys():
            self.common_filters[asset_id].correct()
        self.main_filter.correct()

    def print_filters(self, main_only=False):
        print(str(self.my_id)+"'s Main Filter")
        print(self.main_filter.x_hat)
        print(2*np.sqrt(self.main_filter.P))

        if not main_only:
            print("----------")
            for asset_id in self.common_filters.keys():
                print(str(self.my_id) + "_" + str(asset_id) + " common filter")
                print(self.common_filters[asset_id].x_hat)
                print(2*np.sqrt(self.common_filters[asset_id].P))
                print("----------")

    def _get_implicit_msg_equivalent(self, meas):
        if isinstance(meas, GPSx_Explicit):
            return GPSx_Implicit(meas.R)
        elif isinstance(meas, GPSy_Explicit):
            return GPSy_Implicit(meas.R)
        elif isinstance(meas, LinRelx_Explicit):
            return LinRelx_Implicit(meas.other_asset, meas.R)
        else:
            raise NotImplementedError("Implicit equivalent not found: " + meas.__class__.__name__)

    