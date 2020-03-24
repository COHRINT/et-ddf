from etfilter import *
import numpy as np

class Asset:

    def __init__(self, my_id, x0, P0, A, B, et_delta, red_team=[]):
        # init filters
        self.my_id = my_id

        # Load common filters between other assets
        self.common_filters = {}
        for i in range(x0.size):
            if (i != my_id) and (i not in red_team):
                self.common_filters[i] = ETFilter(my_id, x0, P0, A, B, et_delta)

        self.main_filter = ETFilter_Main(my_id, x0, P0, A, B, et_delta, self.common_filters)
    
    def receive_meas(self, meas, shareable=True):
        self.main_filter.add_meas(self.my_id, meas)

        if shareable and len(self.common_filters) > 0:
            sharing = {}
            for asset_id in self.common_filters.keys():
                common_filter = self.common_filters[asset_id]
                if common_filter.check_implicit(asset_id, meas):
                    sharing[asset_id] = self._get_implicit_msg_equivalent(meas)
                else: # share as explicit
                    sharing[asset_id] = meas
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

    def _get_implicit_msg_equivalent(self, meas):
        if isinstance(meas, GPS_Explicit):
            return GPS_Implicit(meas.R)
        elif isinstance(meas, LinRel_Explicit):
            return LinRel_Implicit(meas.other_asset, meas.R)
        else:
            print("Implicit Msg equivalent not found: " + str(type(meas)))