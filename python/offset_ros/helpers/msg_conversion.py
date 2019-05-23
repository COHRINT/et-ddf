#!/usr/bin/env python

"""
Message conversion utilities to go from ROS messages to python objects.
This is to facilitate keeping the python etddf library under the hood as
seperate as possible from ROS.
"""

import rospy
import numpy as np

from offset.helpers.msg_handling import MeasurementMsg, StateMsg
from offset_etddf.msg import AgentMeasurement, AgentState

def gen_measurement_msg(agent_id,msg):
        """
        Generate generic between-agent measurement message from local measurement
        message.

        Inputs:

            msg -- local measurement message or list of messages

        Returns:

            meas_msg -- generated message or list of messages
        """

        if type(msg) is list:

            meas_msg = []

            for m in msg:

                new_msg = AgentMeasurement()

                # new_msg.type = msg._type.split('/')[1]
                new_msg.header.stamp = rospy.Time.now()
                new_msg.src = agent_id

                if m._type == 'offset_etddf/linrelMeasurement':
                    new_msg.type = 'rel'
                    new_msg.data = [m.x, m.y]
                    new_msg.target = int(m.robot_measured.split("_")[1])
                elif m._type == 'offset_etddf/gpsMeasurement':
                    new_msg.type = 'abs'
                    new_msg.data = [m.x, m.y]

                new_msg.status = [1 for x in new_msg.data]

                meas_msg.append(new_msg)

            return meas_msg

        else:

            meas_msg = AgentMeasurement()

            # meas_msg.type = msg._type.split('/')[1]
            meas_msg.header.stamp = rospy.Time.now()
            meas_msg.src = agent_id

            if msg._type == 'offset_etddf/linrelMeasurement':
                meas_msg.type = 'rel'
                meas_msg.data = [msg.x, msg.y]
                new_msg.target = int(m.robot_measured.split("_")[1])
            elif msg._type == 'offset_etddf/gpsMeasurement':
                meas_msg.type = 'abs'
                meas_msg.data = [msg.x, msg.y]

            meas_msg.status = [1 for x in meas_msg.data]

            return meas_msg

def ros2python_measurement(msg_ros):
    """
    Convert a message from ROS to the corresponding python object.

    Inputs:

        msg_ros -- a single ROS AgentMeasurement message or a list of messages

    Returns:

        msg_python -- a single python MeasurementMsg object or list of objects
    """
    
    if type(msg_ros) is list:

        msg_python = []

        for msg in msg_ros:

            new_msg = MeasurementMsg(msg.src,
                                    msg.dest,
                                    msg.target,
                                    msg.status,
                                    msg.type,
                                    msg.data,
                                    msg.header.stamp.to_sec())

            msg_python.append(new_msg)

        return msg_python

    else:

        msg_python = MeasurementMsg(msg.src,
                                msg.dest,
                                msg.target,
                                msg.status,
                                msg.type,
                                msg.data,
                                msg.header.stamp.to_sec())

        return msg_python

def ros2python_state(msg_ros):
    """
    Convert a message from ROS to the corresponding python object.

    Inputs:

        msg_ros -- a single ROS AgentMeasurement message or a list of messages

    Returns:

        msg_python -- a single python MeasurementMsg object or list of objects
    """
    if type(msg_ros) is list:

        msg_python = []

        for msg in msg_ros:

            inflated_cov = inflate_covariance(msg.covariance)

            new_msg = StateMsg(msg.src,
                                msg.dest,
                                msg.src_meas_connections,
                                msg.src_connections,
                                np.array(msg.mean,ndmin=2).transpose(),
                                inflated_cov,
                                msg.src_ci_rate,
                                msg.header.stamp.to_sec())

            msg_python.append(new_msg)

        return msg_python

    else:

        inflated_cov = inflate_covariance(msg_ros.covariance)

        msg_python = StateMsg(msg_ros.src,
                            msg_ros.dest,
                            msg_ros.src_meas_connections,
                            msg_ros.src_connections,
                            np.array(msg_ros.mean,ndmin=2).transpose(),
                            inflated_cov,
                            msg_ros.src_ci_rate,
                            msg_ros.header.stamp.to_sec())

        return msg_python

def python2ros_measurement(msg_python):
    """
    Convert a message from ROS to the corresponding python object.

    Inputs:

        msg_python -- a single python MeasurementMsg object or list of objects

    Returns:

        msg_ros -- a single ROS AgentMeasurement message or list of messages
    """

    if type(msg_python) is list:

        msg_ros = []

        for msg in msg_python:

            new_msg = AgentMeasurement()
                
            new_msg.src = msg.src
            new_msg.dest = msg.dest
            new_msg.target = msg.target
            new_msg.status = msg.status
            new_msg.type = msg.type_
            new_msg.data = msg.data
            new_msg.header.stamp = rospy.Time.now()

            msg_ros.append(new_msg)

        return msg_ros

    else:

        new_msg = AgentMeasurement()
                
        new_msg.src = msg.src
        new_msg.dest = msg.dest
        new_msg.target = msg.target
        new_msg.status = msg.status
        new_msg.type = msg.type_
        new_msg.data = msg.data
        new_msg.header.stamp = rospy.Time.now()

        return msg_ros

def python2ros_state(msg_python):
    """
    Convert a message from python state object to the corresponding ROS message.

    Inputs:

        msg_python -- a single python StateMsg object or a list of objects

    Returns:

        msg_ros -- a single ROS AgentState message or list of messages
    """
    if type(msg_python) is list:

        msg_ros = []

        for msg in msg_python:

            deflated_cov = deflate_covariance(msg.est_cov)

            new_msg = AgentState()
                
            new_msg.header.stamp = rospy.Time.now()
            new_msg.src = msg.src
            new_msg.dest = msg.dest
            new_msg.src_meas_connections = msg.src_meas_connections
            new_msg.src_connections = msg.src_connections
            new_msg.mean = msg.state_est.flatten().tolist()
            new_msg.covariance = deflated_cov
            new_msg.src_ci_rate = msg.src_ci_rate

            msg_ros.append(new_msg)

        return msg_ros

    else:

        deflated_cov = deflate_covariance(msg_python.est_cov)

        msg_ros = AgentState()
                
        msg_ros.header.stamp = rospy.Time.now()
        msg_ros.src = msg_python.src
        msg_ros.dest = msg_python.dest
        msg_ros.src_meas_connections = msg_python.src_meas_connections
        msg_ros.src_connections = msg_python.src_connections
        msg_ros.mean = msg_python.state_est.flatten().tolist()
        msg_ros.covariance = deflated_cov
        msg_ros.src_ci_rate = msg_python.src_ci_rate

        return msg_ros

def inflate_covariance(covariance_flat):
    """
    Construct full NxN covariance matrix from flattened upper triangular elements.
    Useful for reconstructing covariance data from over the wire, esp. from a ROS msg.

    Inputs:

        covariance -- flattened upper triangular covariance [list]
        cov_size -- dimension of full covariance matrix [int]

    Outputs:

        inflated_covariance -- NxN numpy array representing full matrix
    """
    # compute size of matrix using quadratic formula
    cov_size = int(-1 + np.sqrt(1 + 8*len(covariance_flat))) // 2
    
    # get indicies for upper triangular of correct size
    upper_idx = np.triu_indices(cov_size)
    
    # create empty matrix for storing values
    inflated_covariance = np.empty((cov_size, cov_size))
    
    # fill value in upper triangular, then transpose and fill again
    inflated_covariance[upper_idx] = covariance_flat
    inflated_covariance.T[upper_idx] = covariance_flat
    
    return inflated_covariance

def deflate_covariance(covariance):
    """
    Create flattened upper triangular representation of covariance.
    Useful for sending covariance data over the wire, esp. in a ROS msg.

    Inputs:

        covariance -- NxN covariance matrix in a numpy array

    Returns:

        deflated_covariance -- (N^2 + N)/2 length list of values represented flattened
                                upper triangular covariance
    """

    # create list of indices to grad upper triangular
    upper_tri_idx = np.triu_indices(covariance.shape[0])

    # grab upper triangular elements from covariance using indices
    upper_tri_cov = covariance[upper_tri_idx]

    # convert array to list
    deflated_covariance = upper_tri_cov.tolist()

    return deflated_covariance

def test_deflate_covariance():

    cov = np.array( ( (3,1,2), (1,3,4), (2,4,3)) )

    flat_cov = [3,1,2,3,4,3]

    d_cov = deflate_covariance(cov)

    print(flat_cov)
    print(d_cov)

    assert(flat_cov == d_cov)

def test_inflate_covariance():

    cov = np.array( ( (3,1,2), (1,3,4), (2,4,3)) ,dtype=float)

    flat_cov = [3,1,2,3,4,3]

    i_cov = inflate_covariance(flat_cov)

    print(cov)
    print(i_cov)

    assert(np.array_equal(cov,i_cov))


if __name__ == "__main__":
    # test_deflate_covariance()
    test_inflate_covariance()