#!/usr/bin/env python

"""
Message structure class definitions.
"""

class MeasurementMsg(object):
    """
    Structure for transmitting measurements between agents.
    """
    def __init__(self,src_id,dest_id,target_id,status,type_,data,current_time=None):
        """
        Generates a measurement message instance.

        Inputs:

            src_id      -- id of sending agent (int)
            dest_id     -- id of receiving agent (int)
            target_id   -- id of target agent if relative measurement (int)
            status      -- array indicating which elements of a message are being transmitted (bool array)
            type_       -- measurement type (string)
            data        -- measurement data (float array)
            current_time -- timestamp of message creation
        """
        self.src = src_id
        self.dest = dest_id
        self.target = target_id
        self.status = status
        self.type_ = type_
        self.data = data
        self.timestamp = current_time

class StateMsg(object):
    """
    Structure for transmitting state estimates between agents.
    """
    def __init__(self,src_id,dest_id,src_meas_connections,src_connections,
                    state_est,est_cov,src_ci_rate,current_time=None):
        """
        Generates a state estimate message instance, for use in covariance intersection.
        
        Inputs:

            src_id -- id of sending agent (int)
            dest_id -- id of receving agent (int)
            src_meas_connections -- ids of distance-one (direct) connections (int array)
            src_connections -- ids of all connections (distance-one+) (int array)
            state_est -- mean vector of state estimate (n x 1 numpy array)
            est_cov -- estimate covariance matrix (n x n numpy array)
            src_ci_rate -- sending agent covariance intersection trigger rate (float, [0,1])
            current_time -- timestamp of message creation
        """
        self.src = src_id
        self.dest = dest_id
        self.src_meas_connections = src_meas_connections
        self.src_connections = src_connections
        self.state_est = state_est
        self.est_cov = est_cov
        self.src_ci_rate = src_ci_rate
        self.timestamp = current_time