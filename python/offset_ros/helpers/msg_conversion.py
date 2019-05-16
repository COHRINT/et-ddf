#!/usr/bin/env python

"""
Message conversion utilities to go from ROS messages to python objects.
This is to facilitate keeping the python etddf library under the hood as\
seperate as possible.
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
                                np.array(msg.mean),
                                inflated_cov,
                                msg.src_ci_rate,
                                msg.header.stamp.to_sec())

            msg_python.append(new_msg)

        return msg_python

    else:

        inflated_cov = inflate_covariance(msg.covariance)

        new_msg = StateMsg(msg.src,
                            msg.dest,
                            msg.src_meas_connections,
                            msg.src_connections,
                            np.array(msg.mean),
                            inflated_cov,
                            msg.src_ci_rate,
                            msg.header.stamp.to_sec())

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
            new_msg.type = msg.type
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
        new_msg.type = msg.type
        new_msg.data = msg.data
        new_msg.header.stamp = rospy.Time.now()

        return msg_ros

def python2ros_state(msg_python):
    """
    Convert a message from python state object to the corresponding ROS message.

    Inputs:

        msg -- a single python StateMsg object or a list of objects

    Returns:

        msg_ros -- a single ROS AgentState message or list of messages
    """
    pass

def inflate_covariance(covariance,cov_size):
    """
    Construct full NxN covariance matrix from flattened upper triangular elements.
    Useful for sending covariance data over the wire, esp. in a ROS msg

    Inputs:

        covariance -- flattened upper triangular covariance [list]
        cov_size -- dimension of full covariance matrix [int]

    Outputs:

        inflated_covariance -- NxN numpy array representing full matrix
    """

    # create placeholder covariance
    inflated_covariance = np.zeros( (cov_size,cov_size) )

    el_cnt = 0
    loop_cnt = 0
    while cnt < cov_size:
        for i in range(el_cnt,cov_size):
            inflated_covariance[loop_cnt,el_cnt] = covariance[]
            inflate_covariance[el_cnt,loop_cnt] = 
    