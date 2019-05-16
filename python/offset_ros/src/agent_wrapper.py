#!/usr/bin/env python

"""
ROS wrapper for the Agent class. Manages data messages from sensor topics, 
and messages to and from other agents.
"""

import os
import yaml
import rospy
import Queue as queue
# import queue
import numpy as np
from copy import deepcopy

from offset.agent import Agent
from offset.filters.etkf import ETKF
from offset.dynamics import *

from offset_ros.helpers.msg_conversion import gen_measurement_msg, python2ros_measurement, python2ros_state, ros2python_measurement, ros2python_state

from geometry_msgs.msg import TwistStamped
from offset_etddf.msg import AgentMeasurement, AgentState, gpsMeasurement, linrelMeasurement

class AgentWrapper(object):
    """
    Wraps Agent class by managing data from between-agent messages as well
    as from other agents.

    Inputs:

        config_path -- path to a config file for wrapper (default=None)
    """

    def __init__(self,config_path=None):
        
        # load config file
        # cfg = self.load_config(config_path)

        # set update rate
        # self.update_rate = cfg['update_rate']
        self.agent_name = rospy.get_namespace()
        self.agent_id = rospy.get_param('agent_id')
        self.update_rate = rospy.get_param('agent_update_rate')
        self.connections = rospy.get_param('connections')
        # self.meas_connections = rospy.get_param('meas_connections')
        self.delta = rospy.get_param('delta')
        self.tau_goal = rospy.get_param('tau')
        self.tau = self.tau_goal*0.75
        self.use_adaptive_tau = rospy.get_param('use_adaptive_tau')
        self.epsilon_1 = rospy.get_param('epsilon_1')
        self.epsilon_2 = rospy.get_param('epsilon_2')

        # get agent's initial position, all others assumed 0 w/ large covariance
        start_state = rospy.get_param('start_pos')
        # TODO: add back z and zdot states
        start_state = np.array((start_state['x'],0,start_state['y'],0))

        # create initial control input vector
        self.recent_control_input = np.array([[0],[0]])
        self.recent_twist = np.array([[0],[0]])

        # get availalbe sensors, and noise params
        sensors = rospy.get_param('sensors')
        
        # get dynamics function
        dynamics_fxn = rospy.get_param('dynamics')
        
        # initialize ros node
        rospy.init_node('agent_{}'.format(self.agent_id))

        # create ros rate object for timing update loop
        rate = rospy.Rate(self.update_rate)

        # instantiate Agent object
        # ground_truth = self.init_ground_truth()
        self.agent = self.init_agent(start_state,dynamics_fxn,sensors)
        rospy.loginfo('Agent {} & filters initialized.'.format(self.agent_id))

        # create local and received meaurement queues
        self.local_measurement_queue = queue.LifoQueue()
        self.local_measurement_cnt = 0
        self.received_measurement_queue = queue.LifoQueue()
        self.received_measurement_cnt = 0

        # create local and recevied covariance intersection queue
        self.local_ci_queue = queue.LifoQueue()
        self.recevied_ci_queue = queue.LifoQueue()

        # create subscriber to control input
        # rospy.Subscriber('~actuator_control',ActuatorControl,self.control_input_cb)
        rospy.Subscriber('new_twist',TwistStamped,self.control_input_cb)

        # create subscribers to sensors
        rospy.Subscriber('gps', gpsMeasurement, self.queue_local_measurement)
        rospy.Subscriber('lin_rel', linrelMeasurement, self.queue_local_measurement)

        # create subscribers to comms module
        rospy.Subscriber('agent_meas_incoming', AgentMeasurement, self.queue_received_measurement)
        # rospy.Subscriber('agent_state', AgentState, self.queue_received_state)

        # create publishers to comms module
        self.comms_meas_pub = rospy.Publisher('agent_meas_outgoing', AgentMeasurement, queue_size=10)
        self.comms_state_pub = rospy.Publisher('agent_state_outgoing', AgentState, queue_size=10)

        # create publisher of local esimate
        self.local_est_pub = rospy.Publisher('local_estimate', AgentState, queue_size=10)

        # begin update loop
        while not rospy.is_shutdown():

            # process all queue measurements and ci messages
            self.update()

            # sleep until next update
            rate.sleep()

    def control_input_cb(self,msg):
        """
        Saves most recent control input.
        """
        # NOTE: currently faking this with differencing twist commands
        twist_array = [[msg.twist.linear.x],[msg.twist.linear.y]]
        twist_array = np.array(twist_array)
        # print(twist_array.shape)
        self.recent_control_input = (twist_array - self.recent_twist)/self.update_rate
        # print(self.recent_control_input.shape)
        self.recent_twist = np.array(twist_array)

    def queue_local_measurement(self,msg):
        """
        Add local measurment to queue, and add to count of local messages.
        """
        self.local_measurement_queue.put(msg)
        self.local_measurement_cnt += 1

    def queue_received_measurement(self,msg):
        """
        Add received measurment to queue.
        """
        self.received_measurement_queue.put(msg)
        self.received_measurement_cnt += 1

    def process_local_measurement_queue(self):
        """
        Empty queue and process measurements.
        """
        # get current number of messages in queue
        num_measurements = self.local_measurement_queue.qsize()
        
        # grab the above number of messages in queue
        local_measurements = [self.local_measurement_queue.get() for x in range(num_measurements)]
        
        return local_measurements

    def process_received_measurement_queue(self):
        """
        Empty queue and process measurements.
        """
        # get current number of messages in queue
        num_measurements = self.received_measurement_queue.qsize()
        
        # grab the above number of messages in queue
        received_measurements = [self.received_measurement_queue.get() for x in range(num_measurements)]
        
        return received_measurements

    def local_ci(self):
        """
        Check if covariance intersection needs to happen. If so, generate service requests.
        """
        pass

    def update(self):
        """
        Main update function to process measurements, and ci messages.
        """
        # process measurement queues
        local_measurements = self.process_local_measurement_queue()
        received_measurements = self.process_received_measurement_queue()

        # convert messages from AgentMeasurement and AgentState ROS msg types to
        # MeasurementMsg and StateMsg python msg types (they're pretty much the same)
        local_measurements_generic = gen_measurement_msg(self.agent_id,local_measurements)
        local_measurements_python = ros2python_measurement(local_measurements_generic)
        received_measurements_python = ros2python_measurement(received_measurements)

        # pass measurements to wrapped agent
        # print(local_measurements_generic)
        threshold_measurements = self.agent.process_local_measurements(self.recent_control_input,local_measurements_python)
        # threshold_measurements = self.agent.process_local_measurements(np.array(((0),(0))),local_measurements_python)

        # convert messages back to ros type
        # threshold_measurements_ros = python2ros_measurement(threshold_measurements)

        # published thresholded measurements to connections through comms module
        # if len(local_measurements) > 0:
        #     msg = self.gen_measurement_msg(local_measurements[0])
        #     self.comms_meas_pub.publish(msg)
        # for msg in threshold_measurements_ros:
            # self.comms_meas_pub.pub(msg)

        # self.agent.process_received_measurements(received_measurements_python)

        # check for CI updates

    def init_agent(self,start_state,dynamics_fxn='lin_ncv',sensors={}):
        """
        Create agent, including associated local filters and common information filters.

        Inputs:

            x_true_vec  -- initial true starting positions for every agent
            dynamics    -- name of dynamics fxn to be used
            sensors     -- dictionary of sensors and associated parameters

        Returns:

            agent -- Agent instance
        """
        R_abs = sensors['gps']['noise']
        R_rel = sensors['range_az_el']['noise']

        agent_id = self.agent_id
        ids = sorted(deepcopy(self.connections[agent_id]))
        ids.append(agent_id)

        # build list of distance one and distance two neighbors for each agent
        # each agent gets full list of connections
        neighbor_conn_ids = []
        for j in range(0,len(self.connections[agent_id])):
            for k in range(0,len(self.connections[self.connections[agent_id][j]])):
                # if not any(self.connections[self.connections[agent_id][j]][k] == x for x in neighbor_conn_ids):
                #     neighbor_conn_ids += self.connections[self.connections[agent_id][j]]
                if not self.connections[self.connections[agent_id][j]][k] in neighbor_conn_ids:
                    neighbor_conn_ids += self.connections[self.connections[agent_id][j]]

                # remove agent's own id from list of neighbors
                if agent_id in neighbor_conn_ids:
                    neighbor_conn_ids.remove(agent_id)

        # combine with direct connection ids and sort
        ids = list(set(sorted(ids + neighbor_conn_ids)))

        # divide out direct measurement connections and all connections
        connections_new = list(set(sorted(neighbor_conn_ids + self.connections[agent_id])))
        meas_connections = self.connections[agent_id]
        self.meas_connections = meas_connections

        est_state_length = len(ids)
        print(ids)

        # construct local estimate
        # TODO: remove hardcoded 6
        n = (est_state_length)*4
        F,G,Q = globals()[dynamics_fxn](self.update_rate,est_state_length)
        print('agent {} F shape: {} \t G shape: {} \t Q shape: {}'.format(self.agent_id,F.shape,G.shape,Q.shape))

        # sort all connections
        # ids = sorted([agent_id] + connections_new)
        # create initial state estimate by grabbing relevant ground truth from full ground truth vector
        x0 = np.array([])
        for j in range(0,len(ids)):
            if j == agent_id:
                x = start_state
            else:
                x = np.array((0,0,0,0)).transpose()
            x0 = np.hstack( (x0,x) )

        P0 = 5000*np.eye(4*est_state_length)

        local_filter = ETKF(F,G,0,0,Q,np.array(R_abs),np.array(R_rel),
                            x0.reshape((F.shape[0],1)),P0,self.delta,
                            agent_id,connections_new,-1)

        print(local_filter.connection)

        # construct common information estimates
        common_estimates = []
        for j in range(0,len(meas_connections)):
            
            # find unique states between direct connections
            # inter_states = set(meas_connections).intersection(self.connections[self.connections[i][j]])
            unique_states = set(meas_connections+self.connections[self.connections[agent_id][j]])
            comm_ids = list(unique_states)
            x0_comm = np.array([])
            for k in range(0,len(comm_ids)):
                if k == agent_id:
                    x = start_state
                else:
                    x = np.array((0,0,0,0)).transpose()
                x0_comm = np.hstack( (x0_comm,x) )
            
            # create comm info filter initial covariance
            P0_comm = 5000*np.eye(4*len(comm_ids))

            # generate dynamics
            F_comm, G_comm, Q_comm = globals()[dynamics_fxn](self.update_rate,len(comm_ids))

            # remove agent id from comm ids
            if agent_id in comm_ids:
                comm_ids.remove(agent_id)

            # create common information filter
            comm_filter = ETKF(F_comm,G_comm,0,0,Q_comm,np.array(R_abs),np.array(R_rel),
                                x0_comm.reshape((F_comm.shape[0],1)),P0_comm,self.delta,
                                agent_id,comm_ids,meas_connections[j])

            common_estimates.append(comm_filter)

        # create agent instance
        agent = Agent(agent_id,connections_new,meas_connections,neighbor_conn_ids,
                            local_filter,common_estimates,start_state,
                            0,len(x0)*self.tau_goal,len(x0)*self.tau,
                            self.use_adaptive_tau)

        return agent



if __name__ == "__main__":
    AgentWrapper()