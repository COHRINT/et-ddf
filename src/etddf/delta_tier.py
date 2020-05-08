#!/usr/bin/env python
"""
Minau Delta Tier Asset
"""
from copy import deepcopy
from etddf.ledger_filter import LedgerFilter

class DeltaTier:
    """Windowed Communication Event Triggered Communication

    Provides a buffer that can be pulled and received from another
    DeltaTier filter in a planned fashion. 
    """

    def __init__(self, num_ownship_states, x0, P0, buffer_capacity, meas_space_table, delta_codebook_table, delta_multipliers, asset2id, my_name):
        """Constructor

        Arguments:
            num_ownship_states {int} -- Number of ownship states for each asset
            x0 {np.ndarray} -- initial states
            P0 {np.ndarray} -- initial uncertainty
            buffer_capacity {int} -- capacity of measurement buffer
            meas_space_table {dict} -- Hash that stores how much buffer space a measurement takes up. Str (meas type) -> int (buffer space)
            delta_codebook_table {dict} -- Hash that stores delta trigger for each measurement type. Str(meas type) -> float (delta trigger)
            delta_multipliers {list} -- List of delta trigger multipliers
            asset2id {dict} -- Hash to get the id number of an asset from the string name
            my_name {str} -- Name to loopkup in asset2id the current asset's ID#
        """
        self.ledger_update_times = []
        self.asset2id = asset2id

        # Initialize Common Ledger Filters
        self.delta_tiers = {}
        for multiplier in delta_multipliers:
            self.delta_tiers[multiplier] = LedgerFilter( \
                num_ownship_states, x0, P0, buffer_capacity, \
                buffer_capacity, meas_space_table, \
                multiplier, False, asset2id[my_name]
            )

        # Initialize Main Filter
        self.main_filter = LedgerFilter( \
            num_ownship_states, x0, P0, buffer_capacity, \
            buffer_capacity, meas_space_table, \
            0, True, asset2id[my_name]
        )
        self.num_ownship_states = num_ownship_states


    def add_meas(self, ros_meas):
        """Adds a measurement to all ledger filters

        Arguments:
            ros_meas {etddf.Measurement.msg} -- Measurement taken
        """
        src_id = self.asset2id[ros_meas.src_asset]
        measured_id = self.asset2id[ros_meas.measured_asset]
        self.main_filter.add_meas(ros_meas, src_id, measured_id)
        for key in self.delta_tiers:
            self.delta_tiers[key].add_meas(ros_meas, src_id, measured_id)

    def intersect(self, x, P):
        """Runs covariance intersection and updates internal estimate

        Arguments:
            x {np.ndarray} -- estimate
            P {np.ndarray} -- covariance

        Returns:
            x {np.ndarray} -- intersected estimate
            P {np.ndarray} -- intersected covariance
        """
        pass

    def catch_up(self, shared_buffer):
        """Updates main estimate and common estimate using the shared buffer

        Arguments:
            shared_buffer {list} -- buffer shared from another asset
        """
        # Fill in implicit measurements in the buffer
        # Add all measurements in buffer to ledgers of all ledger_filters
        # Grab lock, no updates for right now
        # Initialize a new main and common filters (all are etfilters) using original estimate
        # Pair the main filter with the etfilter the other asset chose and use to update the main filter
        # Loop through all ledger_update_times
        # update new common etfilter
        # update new main etfilter
        # reset the ledger filters
        pass

    def pull_buffer(self):
        """Pulls lowest delta multiplier's buffer that hasn't overflown

        Returns:
            buffer {list} -- the buffer of ros measurements
        """
        # loop through delta_multipliers, find lowest one that hasn't overflown
        # buffer = flush that buffer
        # delete the other ledgerfilters
        # deepcopy the chosen filter for however many filters were deleted
        # convert the filters using delta_multiplier
        # return 
        pass

    def predict(self, u, Q, time_delta=1.0):
        """Executes prediction step on all filters

        Arguments:
            u {np.ndarray} -- my asset's control input (num_ownship_states / 2, 1)
            Q {np.ndarray} -- motion/process noise (nstates, nstates)

        Keyword Arguments:
            time_delta {float} -- Amount of time to predict in future (default: {1.0})
        """
        self.main_filter.predict(u, Q, time_delta, use_control_input=True)
        for key in self.delta_tiers.keys():
            self.delta_tiers[key].predict(u, Q, time_delta, use_control_input=False)

    def correct(self, update_time):
        """Executes correction step on all filters

        Arguments:
            update_time {time} -- Update time to record on the ledger update times
        """
        self.main_filter.correct(update_time)
        for key in self.delta_tiers.keys():
            self.delta_tiers[key].correct(update_time)

    def get_asset_estimate(self, asset_name):
        """Gets main filter's estimate of an asset

        Arguments:
            asset_name {str} -- Name of asset

        Returns
            np.ndarray -- Mean estimate of asset (num_ownship_states, 1)
            np.ndarray -- Covariance of estimate of asset (num_ownship_states, num_ownship_states)
        """
        asset_id = self.asset2id[asset_name]
        estimate_mean = deepcopy(self.main_filter.filter.x_hat)
        estimate_unc = deepcopy(self.main_filter.filter.P)
        asset_mean = estimate_mean[asset_id*self.num_ownship_states:(asset_id+1)*self.num_ownship_states,0]
        asset_unc = estimate_unc[asset_id*self.num_ownship_states:(asset_id+1)*self.num_ownship_states, \
            self.num_ownship_states:(asset_id+1)*self.num_ownship_states]
        return asset_mean, asset_unc

"""
delta_multipliers := [0.1,0.2,0.5,1]
Initialize delta tier filter

Regular Update (pixhawk OR dvl):
add_meas(depth)
x_new, P_new = intersect(x,P)
publish x_new, P_new to Pixhawk
predict(u, Q, delta_t)
correct(update_time)

Pulling of buffer:
pull_buffer()

Receiving buffer:
catch_up(buffer)

On receival of sonar:
call add_meas()

"""

