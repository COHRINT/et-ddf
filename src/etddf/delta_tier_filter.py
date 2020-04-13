#!/usr/bin/env python
"""
Minau Delta Tier Asset
"""
import copy
from etddf.measurements import *
from etddf.etfilter import *
from etddf.asset import Asset # just utilize the _get_implicit_equivalent() static method

class MinauDeltaTierFilter:

    def __init__(self, num_ownship_states, \
                x0, \
                P0, \
                remaining_meas_buffer_size, \
                meas_buffer_hash, \
                delta_codebook_hash, \
                delta_number, \
                is_main_filter=False, \
                first_timestep=0, \
                meas_ledger=[]):
        self.filter = ETFilter(my_id=0, num_ownship_states=num_ownship_states, world_dim=3, x0=x0, P0=P0, linear_dynamics=False)
        self.num_ownship_states = num_ownship_states
        self.first_timestep = first_timestep
        self.meas_ledger = deepcopy(meas_ledger) # Measurements in et internal form
        self.first_meas_in_timestamp = True
        self.ros_meas_buffer = [] # Measurements as Measurement.msg
        self.ros_meas_buffer_capacity = remaining_meas_buffer_size
        self.meas_buffer_hash = meas_buffer_hash # Hash to get number of units occupied by a measurement
        self.overflown = False
        self.delta_codebook_hash = delta_codebook_hash
        self.delta_number = delta_number
        self.is_main_filter = is_main_filter

    def receive_meas(self, ros_meas, src_id, measured_id):
        # Never receive an implicit measurement this way, do all implicit meas generation in minaudeltatier filter
        et_delta = self._get_meas_et_delta(ros_meas.meas_type)
        meas = self.get_internal_meas_from_ros_meas(ros_meas, src_id, measured_id, et_delta)

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
            self.filter.add_meas(meas)

    def predict(self, u, Q):
        self.filter.predict(u,Q)

    def correct(self):
        self.filter.correct()
        self.first_meas_in_timestamp = True

    def check_overflown(self):
        return self.overflown

    def get_asset_estimate(self, asset_id):
        mean = self.filter.x_hat[asset_id*self.num_ownship_states:(asset_id+1)*self.num_ownship_states,0].reshape(-1,1)
        cov = self.filter.P[asset_id*self.num_ownship_states:(asset_id+1)*self.num_ownship_states,
                            asset_id*self.num_ownship_states:(asset_id+1)*self.num_ownship_states].reshape(self.num_ownship_states, self.num_ownship_states)
        return mean, cov

    def _get_meas_et_delta(self, ros_meas_type):
        if ros_meas_type not in self.delta_codebook_hash.keys():
            raise NotImplementedError("Unrecognized Measurement Type " + ros_meas_type + ". Please add to measurements.yaml")
        else:
            return self.delta_codebook_hash[ros_meas_type]
    
    @staticmethod
    def get_internal_meas_from_ros_meas(ros_meas, src_id, measured_id, et_delta):
        if ros_meas.meas_type == "yaw":
            return GPSyaw_Explicit(src_id, ros_meas.data, ros_meas.variance, et_delta)
        elif ros_meas.meas_type == "depth":
            return GPSz_Explicit(src_id, ros_meas.data, ros_meas.variance, et_delta)
        elif ros_meas.meas_type in ["modem_range","sonar_range"] and not ros_meas.global_pose: # check global_pose list empty
            return Range_Explicit(src_id, measured_id, ros_meas.data, ros_meas.variance, et_delta)
        elif ros_meas.meas_type == "modem_range":
            return RangeFromGlobal_Explicit(measured_id, \
                                            np.array([ros_meas.global_pose]),\
                                            ros_meas.data, ros_meas.variance, et_delta)
        elif ros_meas.meas_type in ["modem_azimuth","sonar_azimuth"] and not ros_meas.global_pose: # check global_pose list empty
            return Azimuth_Explicit(src_id, measured_id, ros_meas.data, ros_meas.variance, et_delta)
        elif ros_meas.meas_type == "modem_azimuth":
            return AzimuthFromGlobal_Explicit(measured_id, \
                                            np.array([ros_meas.global_pose]),\
                                            ros_meas.data, ros_meas.variance, et_delta)
        elif ros_meas.meas_type == "sonar_elevation":
            return Elevation_Explicit(src_id, measured_id, ros_meas.data, ros_meas.variance, et_delta)
        else:
            raise NotImplementedError(str(ros_meas))