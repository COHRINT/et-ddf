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
from pdb import set_trace as st

# TODO add threading protection between functions
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

    def catch_up(self, delta_multiplier, shared_buffer):
        """Updates main estimate and common estimate using the shared buffer

        Arguments:
            delta_multiplier {float} -- multiplier to scale et_delta's with
            shared_buffer {list} -- buffer shared from another asset
        """
        # Fill in implicit measurements in the buffer and align the meas timestamps with our own
        new_buffer = self._fillin_align_buffer(shared_buffer)

        # Add all measurements in buffer to ledgers of all ledger_filters
        for meas in new_buffer:
            self.add_meas(meas)


        # Grab lock, no updates for right now
        # Initialize a new main and common filters (all are etfilters) using original estimate
        # Pair the main filter with the etfilter the other asset chose and use to update the main filter
        # Loop through all ledger_update_times
        # -- for each filter
        # ---- for each meas in measurements add it to
        # update new common etfilter
        # update new main etfilter

        # reset the ledger filters

    def pull_buffer(self):
        """Pulls lowest delta multiplier's buffer that hasn't overflown

        Returns:
            multiplier {float} -- the delta multiplier that was chosen
            buffer {list} -- the buffer of ros measurements
        """
        # Find lowest delta tier that hasn't overflown
        not_overflown_list = []
        for key in self.delta_tiers:
            if not self.delta_tiers[key].check_overflown():
                not_overflown_list.append(key)
            
        # TODO add overflow support -> possibly at runtime rather than extend this callback
        if not not_overflown_list:
            raise NotImplementedError("All deltatier buffers have overflown")
        else:
            lowest_multiplier = min(not_overflown_list)

        last_time = self.main_filter.ledger_update_times[-1]
        buffer = self.delta_tiers[lowest_multiplier].flush_buffer(last_time)
        
        # Change old delta tiers to be copies of the selected one
        for key in self.delta_tiers:
            if key != lowest_multiplier:
                del self.delta_tiers[key]
                self.delta_tiers[key] = deepcopy(self.delta_tiers[lowest_multiplier])
                self.delta_tiers[key].convert(key) # Change the delta multiplier to the new one
                
        return lowest_multiplier, buffer

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

        Raises:
            ValueError: Update time is before the previous update time

        """
        if (len(self.main_filter.ledger_update_times) > 1) and (update_time <= self.main_filter.ledger_update_times[-1]):
            raise ValueError("update time must be later in time than last update time")

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
        # Sort the buffer cronologically
        fxn = lambda x : x.stamp
        shared_buffer.sort(key=fxn)

        new_buffer = []
        expected_meas = {}
        ledger_update_times = self.main_filter.ledger_update_times

        for t in ledger_update_times:
            to_delete = []
            for i in range(len(shared_buffer)):
                meas = shared_buffer[i]

                # Process now if measurment was before ledger time
                if meas.stamp <= t:
                    if "bookstart" in meas.meas_type:
                        meas.meas_type = meas.meas_type[:-len("_bookstart")]
                        expected_meas[meas.meas_type] = [False, meas]
                    # Bookend means stop generating implicit measurments
                    elif "bookend" in meas.meas_type:
                        meas_type = meas.meas_type[:-len("_bookend")]
                        del expected_meas[meas_type]
                    # Final time, no more implicit measurement generation after this
                    elif "final_time" == meas.meas_type:
                        pass
                    # We received a normal explicit measurement
                    else:
                        meas.stamp = t
                        new_buffer.append(meas)
                        expected_meas[meas.meas_type] = [True, meas]

                    # Msg has been processed, add it to be deleted
                    to_delete.append(i)

                # Since buffer is sorted, all following measurements must be at later times
                # so we don't need to search through them at this timestep
                else:
                    break

            # Delete measurements already added to the new buffer
            # Reverse so our indices don't change as we delete elements
            for i in reversed(to_delete):
                shared_buffer.pop(i)
                
            # Fill in implicit measurements at this timestep
            for emeas in expected_meas.keys():
                received, ros_meas = expected_meas[emeas]
                # Generate an implicit measurement
                if not received:
                    new_meas = deepcopy(ros_meas)
                    new_meas.meas_type = emeas + "_implicit"
                    new_meas.stamp = t
                    new_buffer.append(deepcopy(new_meas))
                expected_meas[emeas][0] = False
            
            # If we're out of measurements (including the "final_time"), our buffer is ready
            if len(shared_buffer) == 0:
                break

        return new_buffer

    def debug_print_meas_ledgers(self, multiplier):
        """Prints the measurement ledger of the multiplier's filter

        Arguments:
            multiplier {float} -- Must be a key in self.delta_tiers
        """
        for entry in self.delta_tiers[multiplier].ledger_meas:
            measurements = [x.__class__.__name__ for x in entry]
            print(measurements)

    def debug_common_estimate(self, multiplier):
        """Returns the estimate of the deltatier filter

        Arguments:
            multiplier {float} -- Must be a key in self.delta_tiers

        Returns:
            np.array -- Mean of estimate of the filter
            np.array -- Covariance of estimate of the filter
        """
        return deepcopy(self.delta_tiers[multiplier].x_hat), deepcopy(self.delta_tiers[multiplier].P)

    def debug_print_buffers(self):
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
    meas_space_table = {"depth":2, "dvl_x":2,"dvl_y":2, "bookend":1,"bookstart":1, "final_time":0}
    delta_codebook_table = {"depth":1.0, "dvl_x":1, "dvl_y":1}
    asset2id = {"my_name":0}
    buffer_cap = 10
    dt = DeltaTier(6, x0, P, buffer_cap, meas_space_table, delta_codebook_table, [0.5,1.5], asset2id, "my_name")

    Q = np.eye(6); Q[3:,3:] = np.zeros(Q[3:,3:].shape)
    u = np.array([[0.1,0.1,-0.1]]).T
    t1 = time.time()
    z = Measurement("depth", t1, "my_name","", -1, 0.1, [])
    dvl_x = Measurement("dvl_x", t1, "my_name","", 1, 0.1, [])
    dvl_y = Measurement("dvl_y", t1, "my_name","", 1, 0.1, [])

    test_normal, test_buffer_pull, test_catch_up = False, False, True

    ##### Test Normal Delta Tiering: bookstarts, bookends
    dt.add_meas(z)
    dt.add_meas(dvl_x)
    dt.add_meas(dvl_y)
    dt.predict(u,Q)
    dt.correct(t1)
    if test_normal:
        print("0.5 should have depth,dvl_x,dvl_y")
        print("1.5 should have bookstarts for depth,dvl_x,dvl_y")
        dt.debug_print_buffers()
        print("---")

    t2 = time.time()
    z.stamp = t2
    dvl_x.stamp = t2
    dvl_y.stamp = t2
    dvl_x.data = 2
    dvl_y.data = 2
    dt.add_meas(z)
    dt.add_meas(dvl_x)
    dt.add_meas(dvl_y)
    dt.predict(u,Q)
    dt.correct(t2)
    if test_normal:
        print("0.5 should have depth,dvl_x,dvl_y, dvl_x, dvl_y")
        print("1.5 should have bookstarts{depth,dvl_x,dvl_y}, dvl_x, dvl_y")
        dt.debug_print_buffers()
        print("---")

    t3 = time.time()
    z.stamp = t3
    dvl_x.stamp = t3
    dvl_y.stamp = t3
    z.data = -3
    dt.add_meas(z)
    # dt.add_meas(dvl_x)
    dt.add_meas(dvl_y)
    dt.predict(u,Q)
    dt.correct(t3)
    if test_normal:
        print("0.5 should be overflown")
        print("1.5 should have bookstarts{depth,dvl_x,dvl_y}, dvl_x, dvl_y, depth, dvl_x_bookend")
        dt.debug_print_buffers()
        print("---")

    ##### Test Buffer Pulling #####
    if test_buffer_pull:
        print("0.5 should be overflown")
        print("1.5 should have bookstarts{depth,dvl_x,dvl_y}, dvl_x, dvl_y, depth")
        print("Our buffer should be the 1.5 one")
        dt.debug_print_buffers()
        mult, buffer = dt.pull_buffer()
        strbuffer = [x.meas_type for x in buffer]
        print(strbuffer)
        dt.debug_print_buffers()

    ##### Test catching up #####
    if test_catch_up:
        print("{:.20f}".format(t1))
        print("{:.20f}".format(t2))
        print("{:.20f}".format(t3))
        print("meas ledgers of: " + str(1.5))
        dt.debug_print_meas_ledgers(1.5)
        # dt.debug_print_buffers()
        mult, buffer = dt.pull_buffer()
        buf_contents = [x.meas_type for x in buffer]
        # print(buf_contents)
        from random import shuffle
        shuffle(buffer)
        # TODO, scramble the buffer

        # strbuffer = [x.meas_type for x in buffer]
        # print(strbuffer)

        dt2 = DeltaTier(6, x0, P, buffer_cap, meas_space_table, delta_codebook_table, [0.5,1.5], asset2id, "my_name")
        dt2.predict(u, Q)
        dt2.correct(t1)
        dt2.predict(u, Q)
        dt2.correct(t2)
        dt2.predict(u, Q)
        dt2.correct(t3)

        print("catch up")
        dt2.catch_up(mult, buffer)
        dt2.debug_print_meas_ledgers(1.5)


        # Should see the same meas ledgers as above for both
        # dt2.debug_print_meas_ledgers(0.5)
        # dt2.debug_print_meas_ledgers(1.5)


    ## Pull buffer from one delta tier, instantiate another and predict/correct at all times but with no measurements
    ## Catchup the buffer, the measurement ledgers should be the same
    # the common estimates should be the same, the main estimates should be similar though not the same

    ## Repeat scenario but add a single depth measurement at time t2, see that combining was not a problem
    ## Check measurement ledger and common estimates differ slightly

    ## Test placement of older measurements in the ledger
    ## Test filling in of implicit measurements in the buffer
    ## Test Catching up
    ## Test creation of new ledger filters & main estimate
    ## Check those measurements have actually been fused & the estimate has changed to be more accurate

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