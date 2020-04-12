#!/usr/bin/env python
"""
Minau Delta Tier Asset
"""
import copy
from etddf.measurements import *
from etddf.etfilter import *
from etddf.asset import Asset # just utilize the _get_implicit_equivalent() static method

class MinauDeltaTierFilter:

    def __init__(self, my_id, num_ownship_states, \
                world_dim, \
                x0, \
                P0, \
                linear_dynamics, \
                meas_buffer_size, \
                meas_buffer_hash, \
                delta_codebook_hash, \
                delta_number, \
                is_main_filter=False, \
                first_timestep=0, \
                meas_ledger=[]):
        self.filter = ETFilter(my_id, num_ownship_states, world_dim, x0, P0, linear_dynamics)
        self.first_timestep = first_timestep
        self.meas_ledger = deepcopy(meas_ledger) # Measurements in et internal form
        self.first_meas_in_timestamp = True
        self.ros_meas_buffer = [] # Measurements as Measurement.msg
        self.ros_meas_buffer_capacity = buffer_size
        self.meas_buffer_hash = meas_buffer_hash # Hash to get number of units occupied by a measurement
        self.overflown = False
        self.delta_codebook_hash = delta_codebook_hash
        self.delta_number = delta_number
        self.is_main_filter = is_main_filter

    def receive_meas(self, ros_meas):
        # Look up type of measurement and get et_delta, multiply by delta_factor

        # TODO convert ROS meas to internal meas
        if not self.is_main_filter:
            
            # Check explicit or implicit
            # If explicit add the ros form to our ros_meas_buffer
            # Call Asset's get_implicit_equivalent static method
            meas = None
            if isinstance(meas, Explicit):
                self.ros_meas_buffer.append(ros_meas) # Use meas buffer has to determine if there's room
                # if there's not mark us as overflown
            if self.first_meas_in_timestamp:
                self.meas_ledger.append([meas])
                self.first_meas_in_timestamp = False
            else:
                self.meas_ledger[-1].append(meas)
            

        else:
            # fuse the explicit
            pass

    def predict(self, u, Q):
        self.filter.predict(u,Q)

    def correct(self):
        self.filter.correct()
        self.first_meas_in_timestamp = True

    def check_overflown(self):
        return self.overflown