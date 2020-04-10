#!/usr/bin/env python
"""
Minau Delta Tier Asset
"""


import copy

class MinauDeltaTierAsset:

    def __init__(self, my_id, num_ownship_states, world_dim, x0, P0, linear_dynamics, meas_buffer_size, meas_buffer_hash, first_timestep=0, meas_ledger=[]):
        self.filter = ETFilter(my_id, num_ownship_states, world_dim, x0, P0, linear_dynamics)
        self.first_timestep = first_timestep
        self.meas_ledger = deepcopy(meas_ledger) # Measurements in et internal form
        self.first_meas_in_timestamp = True
        self.ros_meas_buffer = [] # Measurements as Measurement.msg
        self.ros_meas_buffer_capacity = buffer_size
        self.meas_buffer_hash = meas_buffer_hash # Hash to get number of units occupied by a measurement

    def receive_meas(self, ros_meas, is_main=False):
        # TODO convert ROS meas to internal meas then add to filter
        if not is_main:
            
            # Check explicit or implicit
            # If explicit add the ros form to our ros_meas_buffer
            # Call asset's _get_implicit_equivalent method
            meas = None
            if self.first_meas_in_timestamp:
                self.meas_ledger.append([])
                self.first_meas_in_timestamp = False
            else:
                self.meas_ledger[-1].append()
            

        else:
            # fuse the explicit
            pass

    def predict(self, u, Q):
        self.filter.predict(u,Q)

    def correct(self):
        self.filter.correct()
        self.first_meas_in_timestamp = True