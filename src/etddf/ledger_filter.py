#!/usr/bin/env python
"""Contains class that records filter inputs and makes available an event triggered buffer.

LedgerFilter
"""
from copy import deepcopy
from etddf.etfilter import ETFilter, ETFilter_Main
from etddf.measurements import *
from etddf.asset import Asset
import numpy as np

MEASUREMENTS_TYPES_NOT_SHARED=["modem"]
FUSE_MEAS_NEXT_UPDATE = -1

class LedgerFilter:
    """Records filter inputs and makes available an event triggered buffer. """
    
    ledger_meas = [] # In internal measurement form
    ledger_control = [] ## elements with [u, Q, time_delta, use_control_input]
    ledger_ci = [] ## Covariance Intersection ledger, each element is of form [x, P]
    ledger_update_times = [] ## Update times of when correction step executed
    expected_measurements = {} # When we don't receive an expected measurement we need to insert a "bookend" into the measurement buffer

    def __init__(self, num_ownship_states, x0, P0, buffer_capacity, meas_space_table, delta_codebook_table, delta_multiplier, is_main_filter, my_id):
        """Constructor

        Arguments:
            num_ownship_states {int} -- Number of ownship states for each asset
            x0 {np.ndarray} -- initial states
            P0 {np.ndarray} -- initial uncertainty
            buffer_capacity {int} -- capacity of measurement buffer
            meas_space_table {dict} -- Hash that stores how much buffer space a measurement takes up. Str (meas type) -> int (buffer space)
            delta_codebook_table {dict} -- Hash that stores delta trigger for each measurement type. Str(meas type) -> float (delta trigger)
            delta_multiplier {float} -- Delta trigger constant multiplier for this filter
            is_main_filter {bool} -- Is this filter a common or main filter (if main the meas buffer does not matter)
            my_id {int} -- ID# of the current asset (typically 0)
        """
        self.original_estimate = [deepcopy(x0), deepcopy(P0)]
        self.delta_codebook_table = delta_codebook_table
        self.delta_multiplier = delta_multiplier
        self.buffer = MeasurementBuffer(meas_space_table, buffer_capacity)
        self.is_main_filter = is_main_filter
        self.filter = ETFilter(my_id, num_ownship_states, 3, x0, P0, True)

        # Initialize first element of ledgers
        self.ledger_meas.append([])
        self.ledger_control.append([])
        self.ledger_ci.append([])
        self.ledger_update_times.append([])

    def add_meas(self, ros_meas, src_id, measured_id, time_index=FUSE_MEAS_NEXT_UPDATE):        
        """Adds and records a measurement to the filter at the specified time

        Will not attempt to track measurements

        Arguments:
            ros_meas {etddf.Measurement.msg} -- The measurement in ROS form
            src_id {int} -- asset ID that took the measurement
            measured_id {int} -- asset ID that was measured (can be any value for ownship measurement)

        Keyword Arguments:
            time_index {int} -- time of the ros measurement taken (default: {FUSE_MEAS_NEXT_UPDATE})
                measurements for times not now will be recorded on the ledger but not fused
        """
        # Get the delta trigger for this measurement
        et_delta = self._get_meas_et_delta(ros_meas.meas_type)

        # Convert ros_meas to an implicit or explicit internal measurement
        meas = self._get_internal_meas_from_ros_meas(ros_meas, src_id, measured_id, et_delta)
        orig_meas = meas

        # Common filter with delta tiering
        if not self.is_main_filter and time_index==FUSE_MEAS_NEXT_UPDATE:

            # Check for Implicit Update
            if self.filter.check_implicit(meas):
                meas = Asset.get_implicit_msg_equivalent(meas)
            else:
                # TODO Check for overflow
                self.buffer.add_meas(ros_meas)
            
            # Implicit Bookend Logic
            # Check if we should be expecting more of these measurements
            l = [x for x in MEASUREMENTS_TYPES_NOT_SHARED if x in ros_meas.meas_type]
            if not l and not isinstance(orig_meas, Implicit):
                self.expected_measurements[type(orig_meas)] = [True, ros_meas]

        # Append to the ledger
        self.ledger_meas[time_index].append(meas)

        # Fuse on next timestamp
        if time_index == FUSE_MEAS_NEXT_UPDATE:
            self.filter.add_meas(meas)
        else:
            pass # Measurement will be fused on catch up

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
                print("Did not receive expected measurement: " + str(emeas))
                self.buffer.insert_bookend(ros_meas, update_time)
                del self.expected_measurements[emeas]
            # Received our expected measurement
            else:
                self.expected_measurements[emeas] = [False, ros_meas]

        # Run correction step on filter
        self.filter.correct()
        self.ledger_update_times[-1] = update_time

        # Initialize next element of ledgers
        self.ledger_meas.append([])
        self.ledger_control.append([])
        self.ledger_ci.append([])
        self.ledger_update_times.append([])

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
        found = [x for x in self.delta_codebook_table.keys() if x in ros_meas.meas_type]
        if not found:
            raise KeyError("Unrecognized Measurement Type " + ros_meas.meas_type + ". Please add to etddf/measurements.py")
        else:
            return self.delta_codebook_table[ros_meas.meas_type] * self.delta_multiplier
    
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

class MeasurementBuffer:
    """ Manages a delta tier buffer for windowed commmunication """

    buffer = []
    size = 0
    meas_space_table = None
    overflown = False
    capacity = 0

    def __init__(self, meas_space_table, capacity):
        """Constructor

        Arguments:
            meas_space_table {dict} -- Hash to get how much buffer space a measurement takes up. Str (meas type) -> int (buffer space)
            capacity {int} -- capacity of the buffer
        """
        self.meas_space_table = meas_space_table
        self.capacity = capacity

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
        meas_space = self.meas_space_table[ros_meas.meas_type]
        self.size += meas_space
        if self.size > self.capacity:
            self.overflown = True
        return self.overflown

    def flush(self):
        """Returns and clears the buffer
        
        Returns:
            buffer {list} -- the flushed buffer
        """
        old_buffer = deepcopy(self.buffer)
        self.buffer = []
        self.overflown = False
        self.size = 0
        return old_buffer

    def insert_bookend(self, ros_meas, timestamp):
        """Inserts a bookend to the buffer
        A bookend indicates the halt of a measurement stream. This lets the other asset
        receiving the buffer know to stop generating implicit measurments in between explicit ones

        Arguments:
            ros_meas {etddf.Measurement.msg} -- A ros measurement of the type to be halted
            timestamp {time} -- Timestamp to indicate halt of implicit measurements

        Returns:
            bool -- buffer overflow indicator
        """
        bookend = deepcopy(ros_meas)
        bookend.meas_type += "_bookend"
        bookend.stamp = timestamp
        return self.add_meas(bookend)