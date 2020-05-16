from __future__ import division
"""@package etddf

Contains class that records filter inputs and makes available an event triggered buffer.

"""
__author__ = "Luke Barbier"
__copyright__ = "Copyright 2020, COHRINT Lab"
__email__ = "luke.barbier@colorado.edu"
__status__ = "Development"
__license__ = "MIT"
__maintainer__ = "Luke Barbier"
__version__ = "1.0.0"

from copy import deepcopy
from etddf.etfilter import ETFilter, ETFilter_Main
from etddf.measurements import *
from etddf.asset import Asset
import numpy as np
from pdb import set_trace as st
from etddf.msg import Measurement

## Lists all measurement substrings to not send implicitly
MEASUREMENT_TYPES_NOT_SHARED =     ["modem"]
## Indicates meas should be fused on upcoming update step
FUSE_MEAS_NEXT_UPDATE =             -1
## Indicates to use this filters delta multiplier (Won't be used when fusing a buffer from another asset)
THIS_FILTERS_DELTA =                -1

class LedgerFilter:
    """Records filter inputs and makes available an event triggered buffer. """    

    def __init__(self, num_ownship_states, x0, P0, buffer_capacity, meas_space_table, delta_codebook_table, delta_multiplier, is_main_filter, my_id):
        """Constructor

        Arguments:
            num_ownship_states {int} -- Number of ownship states for each asset
            x0 {np.ndarray} -- initial states
            P0 {np.ndarray} -- initial uncertainty
            buffer_capacity {int} -- capacity of measurement buffer
            meas_space_table {dict} -- Hash that stores how much buffer space a measurement takes up. Str (meas type) -> int (buffer space)
                Must have key entries "bookend", "bookstart" to indicate space needed for measurement implicitness filling in
            delta_codebook_table {dict} -- Hash that stores delta trigger for each measurement type. Str(meas type) -> float (delta trigger)
            delta_multiplier {float} -- Delta trigger constant multiplier for this filter
            is_main_filter {bool} -- Is this filter a common or main filter (if main the meas buffer does not matter)
            my_id {int} -- ID# of the current asset (typically 0)
        """
        if delta_multiplier <= 0:
            raise ValueError("Delta Multiplier must be greater than 0")

        self.original_estimate = [deepcopy(x0), deepcopy(P0)]
        self.delta_codebook_table = delta_codebook_table
        self.delta_multiplier = delta_multiplier
        self.buffer = MeasurementBuffer(meas_space_table, buffer_capacity)
        self.is_main_filter = is_main_filter
        self.filter = ETFilter(my_id, num_ownship_states, 3, x0, P0, True)

        # Initialize Ledgers
        self.ledger_meas = [] # In internal measurement form
        self.ledger_control = [] ## elements with [u, Q, time_delta, use_control_input]
        self.ledger_ci = [] ## Covariance Intersection ledger, each element is of form [x, P]
        self.ledger_update_times = [] ## Update times of when correction step executed
        self.expected_measurements = {} # When we don't receive an expected measurement we need to insert a "bookend" into the measurement buffer

        # Initialize first element of ledgers
        self.ledger_meas.append([])
        self.ledger_control.append([])
        self.ledger_ci.append([])

    def add_meas(self, ros_meas, src_id, measured_id, delta_multiplier=THIS_FILTERS_DELTA):
        """Adds and records a measurement to the filter

        Measurements after last correction step time will be fused at next correction step
            measurements before will be recorded and fused in catch_up()

        Arguments:
            ros_meas {etddf.Measurement.msg} -- The measurement in ROS form
            src_id {int} -- asset ID that took the measurement
            measured_id {int} -- asset ID that was measured (can be any value for ownship measurement)

        Keyword Arguments:
            delta_multiplier {int} -- Delta multiplier to use for this measurement (default: {THIS_FILTERS_DELTA})
        """
        # Get the delta trigger for this measurement
        et_delta = self._get_meas_et_delta(ros_meas)

        # Check if we're not using a standard delta_multiplier
        if delta_multiplier != THIS_FILTERS_DELTA:
            et_delta = (et_delta / self.delta_multiplier) * delta_multiplier # Undo delta multiplication and scale et_delta by new multiplier

        # Get the update time index of the measurement
        time_index = self._get_time_index(ros_meas.stamp)

        # Convert ros_meas to an implicit or explicit internal measurement
        meas = self._get_internal_meas_from_ros_meas(ros_meas, src_id, measured_id, et_delta)
        orig_meas = meas

        # Common filter with delta tiering
        if not self.is_main_filter and time_index==FUSE_MEAS_NEXT_UPDATE:

            # Check if this measurement is allowed to be sent implicitly
            l = [x for x in MEASUREMENT_TYPES_NOT_SHARED if x in ros_meas.meas_type]
            if not l:
                # Check for Implicit Update
                if self.filter.check_implicit(meas):

                    # Check if this is the first of the measurement stream, if so, insert a bookstart
                    bookstart = meas.__class__.__name__ not in self.expected_measurements.keys()
                    
                    if bookstart:
                        self.buffer.insert_marker(ros_meas, ros_meas.stamp, bookstart=True)

                    meas = Asset.get_implicit_msg_equivalent(meas)
                    
                # Fuse explicitly
                else:
                    # TODO Check for overflow
                    self.buffer.add_meas(deepcopy(ros_meas))
                
                # Indicate we receieved our expected measurement
                self.expected_measurements[orig_meas.__class__.__name__] = [True, ros_meas]

        # Append to the ledger
        self.ledger_meas[time_index].append(meas)

        # Fuse on next timestamp
        if time_index == FUSE_MEAS_NEXT_UPDATE:
            self.filter.add_meas(meas)
        else:
            pass # Measurement will be fused on delta_tier's catch_up()

    def predict(self, u, Q, time_delta=1.0, use_control_input=False):
        """Executes filter's prediction step

        Arguments:
            u {np.ndarray} -- control input (num_ownship_states / 2, 1)
            Q {np.ndarray} -- motion/process noise (nstates, nstates)

        Keyword Arguments:
            time_delta {float} -- Amount of time to predict in future (default: {1.0})
            use_control_input {bool} -- Whether to use control input or assume constant velocity (default: {False})
        """
        self.filter.predict(u, Q, time_delta, use_control_input)
        self.ledger_control[-1] = [u, Q, time_delta, use_control_input]

    def correct(self, update_time):
        """Execute Correction Step in filter

        Arguments:
            update_time {time} -- Update time to record on the ledger update times
        """

        # Check if we received all of the measurements we were expecting
        for emeas in self.expected_measurements.keys():
            [rx, ros_meas] = self.expected_measurements[emeas]

            # Did not receive our expected measurement
            if not rx:
                print("Did not receive expected measurement: " + emeas)
                self.buffer.insert_marker(ros_meas, update_time, bookstart=False)
                del self.expected_measurements[emeas]
            # Received our expected measurement
            else:
                self.expected_measurements[emeas] = [False, ros_meas]

        # Run correction step on filter
        self.filter.correct()
        self.ledger_update_times.append(update_time)

        # Initialize next element of ledgers
        self.ledger_meas.append([])
        self.ledger_control.append([])
        self.ledger_ci.append([])

    def convert(self, delta_multiplier):
        """Converts the filter to have a new delta multiplier
            
        Arguments:
            delta_multiplier {float} -- the delta multiplier of the new filter
        """
        self.delta_multiplier = delta_multiplier

    def check_overflown(self):
        """Checks whether the filter's buffer has overflown

        Returns:
            bool -- True if buffer has overflown
        """
        return self.buffer.check_overflown()

    def flush_buffer(self, final_time):
        """Returns the event triggered buffer

        Arguments:
            final_time {time} -- the last time measurements were considered to be added to the buffer 

        Returns:
            list -- the flushed buffer of measurements
        """
        return self.buffer.flush(final_time)

    def reset(self, buffer, ledger_update_times, ledger_meas, ledger_control=None, ledger_ci=None):
        """Resets a ledger filter with the inputted ledgers

        Arguments:
            buffer {MeasurementBuffer} -- Measurement buffer to be preserved
            ledger_update_times {list} -- Update times
            ledger_meas {list} -- List of measurements at each update time

        Keyword Arguments:
            ledger_control {list} -- List of control inputs (default: {None})
            ledger_ci {list} -- List of covariance intersections (default: {None})

        Raises:
            ValueError: lengths of ledgers do not match
        """
        self.buffer = deepcopy(buffer)
        self.ledger_update_times = deepcopy(ledger_update_times)

        # Measurement Ledger
        self.ledger_meas = deepcopy(ledger_meas)
        if len(self.ledger_meas)-1 != len(self.ledger_update_times):
                raise ValueError("Meas Ledger does not match length of update times!")

        # Control Input Ledger
        if ledger_control is not None:
            self.ledger_control = ledger_control
            if len(self.ledger_control)-1 != len(self.ledger_update_times):
                raise ValueError("Control Ledger does not match length of update times!")
        else:
            # Initialize with empty lists
            self.ledger_control = [[] for _ in range(len(self.ledger_update_times))]

        # Covariance intersection ledger
        if ledger_ci is not None:
            self.ledger_ci = ledger_ci
            if len(self.ledger_ci)-1 != len(self.ledger_update_times):
                raise ValueError("CI Ledger does not match length of update times!")
        else:
            # Initialize with empty lists
            self.ledger_ci = [[] for _ in range(len(self.ledger_update_times))]
        
    def _get_meas_et_delta(self, ros_meas):
        """Gets the delta trigger for the measurement

        Arguments:
            ros_meas {etddf.Measurement.msg} -- The measurement in ROS form

        Raises:
            KeyError: ros_meas.meas_type not found in the delta_codebook_table

        Returns:
            float -- the delta trigger scaled by the filter's delta multiplier
        """
        # Match root measurement type e.g. "modem_range" with "modem_range_implicit"
        for meas_type in self.delta_codebook_table.keys():
            if meas_type in ros_meas.meas_type:
                return self.delta_codebook_table[meas_type] * self.delta_multiplier
        raise KeyError("Measurement Type " + ros_meas.meas_type + " not found in self.delta_codebook_table")
    
    def _get_internal_meas_from_ros_meas(self, ros_meas, src_id, measured_id, et_delta):
        """Converts etddf/Measurement.msg (implicit or explicit) to a class in etddf/measurements.py

        Arguments:
            ros_meas {etddf.Measurement.msg} -- The measurement in ROS form
            src_id {int} -- asset ID that took the measurement
            measured_id {int} -- asset ID that was measured (can be any value for ownship measurement)
            et_delta {float} -- Delta trigger for this measurement

        Raises:
            NotImplementedError: Conversion between measurements forms has not been specified

        Returns:
            etddf.measurements.Explicit -- measurement in filter's internal form
        """
        if "implicit" not in ros_meas.meas_type:
            if ros_meas.meas_type == "depth":
                return GPSz_Explicit(src_id, ros_meas.data, ros_meas.variance, et_delta)
            elif ros_meas.meas_type == "modem_range" and not ros_meas.global_pose: # check global_pose list empty
                return Range_Explicit(src_id, measured_id, ros_meas.data, ros_meas.variance, et_delta)
            elif ros_meas.meas_type == "modem_range":
                return RangeFromGlobal_Explicit(measured_id, \
                                                np.array([ros_meas.global_pose]),\
                                                ros_meas.data, ros_meas.variance, et_delta)
            elif ros_meas.meas_type == "modem_azimuth" and not ros_meas.global_pose: # check global_pose list empty
                return Azimuth_Explicit(src_id, measured_id, ros_meas.data, ros_meas.variance, et_delta)
            elif ros_meas.meas_type == "modem_azimuth":
                return AzimuthFromGlobal_Explicit(measured_id, \
                                                np.array([ros_meas.global_pose]),\
                                                ros_meas.data, ros_meas.variance, et_delta)
            elif ros_meas.meas_type == "dvl_x":
                return Velocityx_Explicit(src_id, ros_meas.data, ros_meas.variance, et_delta)
            elif ros_meas.meas_type == "dvl_y":
                return Velocityy_Explicit(src_id, ros_meas.data, ros_meas.variance, et_delta)
            else:
                raise NotImplementedError(str(ros_meas))
        # Implicit Measurement
        else:
            if ros_meas.meas_type == "depth_implicit":
                return GPSz_Implicit(src_id, ros_meas.variance, et_delta)
            elif ros_meas.meas_type == "dvl_x_implicit":
                return Velocityx_Implicit(src_id, ros_meas.variance, et_delta)
            elif ros_meas.meas_type == "dvl_y_implicit":
                return Velocityy_Implicit(src_id, ros_meas.variance, et_delta)
            else:
                raise NotImplementedError(str(ros_meas))
    
    def _get_time_index(self, time_):
        """Converts a time to an update time index

        Uses the ledger of correction step times

        Arguments:
            time_ {time} -- Time to convert

        Returns:
            int -- corresponding index in the measurement,ci and control ledger
                if time_ > last update time --> returns FUSE_MEAS_NEXT_UPDATE
        """
        # Check if we're fusing at next measurement time
        if len(self.ledger_update_times) == 0 or time_ > self.ledger_update_times[-1]:
            return FUSE_MEAS_NEXT_UPDATE
        
        # Lookup on what index this corresponds to in the ledger_update_times
        # Note: len(ledger_meas/control/ci) is always 1 greater than len(ledger_update_times)
        # Therefore, the first if statement in this fxn is for filling that last/newest slot
        # This for loop then has a 1 to 1 correspondence: ledger_update_times[i] corresponds to ledger_meas/control/ci[i]
        # The subtraction and addition below of indices is for zero order holding all measurements between times
        # t1 and t2 to be associated with time t2
        for ind in reversed(range(len(self.ledger_update_times) - 1)):
            if time_ > self.ledger_update_times[ind]:
                return ind + 1
        return 0

class MeasurementBuffer:
    """ Manages a delta tier buffer for windowed commmunication """

    def __init__(self, meas_space_table, capacity):
        """Constructor

        Arguments:
            meas_space_table {dict} -- Hash to get how much buffer space a measurement takes up. Str (meas type) -> int (buffer space)
            capacity {int} -- capacity of the buffer
        """
        self.meas_space_table = meas_space_table

        # Make room for the final time marker
        self.capacity = capacity - meas_space_table["final_time"]

        self.buffer = []
        self.size = 0
        self.overflown = False

    def check_overflown(self):
        """Checks whether the filter's buffer has overflown

        Returns:
            bool -- True if buffer has overflown
        """
        return self.overflown

    def add_meas(self, ros_meas):
        """Adds a measurement to the buffer, checks for overflow

        Arguments:
            ros_meas {etddf.Measurement.msg} -- the measurement to add

        Returns:
            overflown {bool} -- whether the buffer has overflown
        """
        self.buffer.append(ros_meas)
        if "bookstart" in ros_meas.meas_type:
            meas_space = self.meas_space_table["bookstart"]
        elif "bookend" in ros_meas.meas_type:
            meas_space = self.meas_space_table["bookend"]
        else:
            meas_space = self.meas_space_table[ros_meas.meas_type]
        self.size += meas_space
        if self.size > self.capacity:
            self.overflown = True
        return self.overflown

    def flush(self, final_time):
        """Returns and clears the buffer

        Arguments:
            final_time {time} -- the last time measurements were considered to be added to the buffer 
        
        Returns:
            buffer {list} -- the flushed buffer
        """
        # Insert the "final_time" marker at the end of the buffer
        final_time_marker = Measurement("final_time", final_time, "","",0.0,0.0,[])
        self.buffer.append(final_time_marker)

        old_buffer = deepcopy(self.buffer)

        # Clear buffer
        self.buffer = []
        self.overflown = False
        self.size = 0

        return old_buffer

    def insert_marker(self, ros_meas, timestamp, bookstart=True):
        """Inserts a bookstart/bookend to the buffer

        A bookstart indicates the start of a measurement stream. This lets the other asset
        receiving the buffer know to generate implicit measurments until the bookend (or end of buffer). 
        Measurement streams are usually started by an explicit measurement, but can be started by
        bookstarts if the bookstart type takes less space.

        A bookend indicates the halt of a measurement stream. This lets the other asset
        receiving the buffer know to stop generating implicit measurments in between explicit ones

        Arguments:
            ros_meas {etddf.Measurement.msg} -- A ros measurement of the type to be halted
            timestamp {time} -- Timestamp to indicate halt of implicit measurements

        Keyword Arguments:
            bookstart {bool} -- whether to generate a bookstart or bookend (default: {True})

        Returns:
            bool -- buffer overflow indicator
        """
        marker = deepcopy(ros_meas)
        if bookstart:
            marker.meas_type += "_bookstart"
        else:
            marker.meas_type += "_bookend"
        marker.stamp = timestamp
        return self.add_meas(marker)