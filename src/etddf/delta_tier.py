"""@package etddf

Delta Tier class for windowed event triggered communication

"""
__author__ = "Luke Barbier"
__copyright__ = "Copyright 2020, COHRINT Lab"
__email__ = "luke.barbier@colorado.edu"
__status__ = "Development"
__license__ = "MIT"
__maintainer__ = "Luke Barbier"
__version__ = "1.0.0"

from copy import deepcopy
from etddf.ledger_filter import LedgerFilter, MEASUREMENT_TYPES_NOT_SHARED
from etddf.msg import Measurement
import time

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
            self.delta_tiers[multiplier] = LedgerFilter(
                num_ownship_states, x0, P0, \
                buffer_capacity, meas_space_table, \
                delta_codebook_table, multiplier, \
                False, asset2id[my_name]
            )

        # Initialize Main Filter
        self.main_filter = LedgerFilter(
            num_ownship_states, x0, P0, \
            buffer_capacity, meas_space_table, \
            delta_codebook_table, multiplier, \
            True, asset2id[my_name]
        )
        
        self.num_ownship_states = num_ownship_states


    def add_meas(self, ros_meas):
        """Adds a measurement to all ledger filters. Measurement will be fused on next correction step

        Arguments:
            ros_meas {etddf.Measurement.msg} -- Measurement taken
        """
        src_id = self.asset2id[ros_meas.src_asset]
        if ros_meas.measured_asset in self.asset2id.keys():
            measured_id = self.asset2id[ros_meas.measured_asset]
        else:
            measured_id = -1
        self.main_filter.add_meas(ros_meas, src_id, measured_id)
        key = self.delta_tiers.keys()[0]
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

    def catch_up(self, shared_buffer, delta_multiplier):
        """Updates main estimate and common estimate using the shared buffer

        Arguments:
            shared_buffer {list} -- buffer shared from another asset
            delta_multiplier {float} -- multiplier to scale et_delta's with
        """
        # Fill in implicit measurements in the buffer and align the meas timestamps with our own
        new_buffer, time_indices = self._fillin_align_buffer(shared_buffer)
        # Add all measurements in buffer to ledgers of all ledger_filters

        # for meas in new_buffer:
        #     self.main_filter.add_meas()

        # Grab lock, no updates for right now
        # Initialize a new main and common filters (all are etfilters) using original estimate
        # Pair the main filter with the etfilter the other asset chose and use to update the main filter
        # Loop through all ledger_update_times
        # update new common etfilter
        # update new main etfilter
        # reset the ledger filters

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

    def _fillin_align_buffer(self, shared_buffer):
        """Fills in implicit measurements to buffer and generates time indices

        Arguments:
            shared_buffer {list} -- List of measurements received from other asset
        """
        new_buffer = []
        expected_meas = {}
        current_time = shared_buffer[0].stamp
        for meas in shared_buffer:
            if "bookstart" in meas.meas_type:
                expected_meas[meas.meas_type[:-len("_bookstart")]] = meas
            elif "bookend" in meas.meas_type:
                del expected_meas[meas.meas_type[:-len("_bookend")]]
            
            if current_time > meas.stamp:
                # Fill in all implicit measurements
                for emeas in expected_meas.keys():
                    new_implicit = deepcopy(expected_meas[emeas])
                    new_implicit.meas_type = meas.meas_type[:-len("_bookstart")] + "_implicit"
                    new_implicit.stamp = current_time
                    new_buffer.append(new_implicit)
                current_time = meas.stamp

    def debug_print_deltatiers(self):
        """Prints the contents of all of the delta tier buffers
        """
        for dt_key in self.delta_tiers.keys():
            buffer = [x.meas_type for x in self.delta_tiers[dt_key].buffer.buffer]
            if self.delta_tiers[dt_key].check_overflown():
                print(str(dt_key) + " is overflown")
            print(str(dt_key) + " " + str(buffer))

if __name__ == "__main__":
    # Test plumbing
    import numpy as np
    x0 = np.zeros((6,1))
    P = np.ones((6,6)) * 10
    meas_space_table = {"depth":2, "dvl_x":2,"dvl_y":2, "bookend":1,"bookstart":1}
    delta_codebook_table = {"depth":1.0, "dvl_x":1, "dvl_y":1}
    asset2id = {"my_name":0}
    buffer_cap = 10
    dt = DeltaTier(6, x0, P, buffer_cap, meas_space_table, delta_codebook_table, [0.5,1.5], asset2id, "my_name")

    # Test regular update
    ## Init process noise & contol input
    Q = np.eye(6); Q[3:,3:] = np.zeros(Q[3:,3:].shape)
    u = np.array([[0.1,0.1,-0.1]]).T
    t = time.time()
    z = Measurement("depth", t, "my_name","", -1, 0.1, [])
    dvl_x = Measurement("dvl_x", t, "my_name","", 1, 0.1, [])
    dvl_y = Measurement("dvl_y", t, "my_name","", 1, 0.1, [])

    # First loop should add fill up first buffer but not second
    print("0.5 should have depth,dvl_x,dvl_y")
    print("1.5 should have bookstarts for depth,dvl_x,dvl_y")
    dt.add_meas(z)
    dt.add_meas(dvl_x)
    dt.add_meas(dvl_y)
    dt.predict(u,Q)
    dt.correct(t)
    dt.debug_print_deltatiers()
    print("---")

    print("0.5 should have depth,dvl_x,dvl_y, dvl_x, dvl_y")
    print("1.5 should have bookstarts{depth,dvl_x,dvl_y}, dvl_x, dvl_y")
    t = time.time()
    z.stamp = t
    dvl_x.stamp = t
    dvl_y.stamp = t
    dvl_x.data = 2
    dvl_y.data = 2
    dt.add_meas(z)
    dt.add_meas(dvl_x)
    dt.add_meas(dvl_y)
    dt.predict(u,Q)
    dt.correct(t)
    dt.debug_print_deltatiers()
    print("---")

    print("0.5 should be overflown")
    print("1.5 should have bookstarts{depth,dvl_x,dvl_y}, dvl_x, dvl_y, depth")
    t = time.time()
    z.stamp = t
    dvl_x.stamp = t
    dvl_y.stamp = t
    z.data = -3
    dt.add_meas(z)
    dt.add_meas(dvl_x)
    dt.add_meas(dvl_y)
    dt.predict(u,Q)
    dt.correct(t)
    dt.debug_print_deltatiers()
    print("---")

    
    # TODO Loop and add truth measurements
    # for i in range()
    ### LOOP below until first delta tier has overflown
    ## Add depth meas
    ## Add dvl_x, dvl_y
    ## Predict
    ## Correct
    ### END LOOP
    ## Add new depth, add new dvl_x, dvl_y that are "surprising for some filters"
    ## predict
    ## correct
    ## Check predicted 
    # Pull buffer
    # Create a new buffer
    # Call catchup with new buffer
    # Check estimate has changed but is still in same ballpark
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