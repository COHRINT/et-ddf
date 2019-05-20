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

from offset_ros.helpers.msg_conversion import *

from geometry_msgs.msg import TwistStamped
from offset_etddf.msg import AgentMeasurement, AgentState, gpsMeasurement, linrelMeasurement
from offset_etddf.srv import CIUpdate

ROS_LOG_LEVEL = rospy.INFO

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
        rospy.init_node('agent_{}'.format(self.agent_id),log_level=ROS_LOG_LEVEL)

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
        # self.local_ci_queue = queue.LifoQueue()
        # self.recevied_ci_queue = queue.LifoQueue()
        self.ci_queue = queue.LifoQueue()
        self.ci_cnt = 0

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

        # create CI update service
        self.ci_srv = rospy.Service('ci_update',CIUpdate,self.ci_service_handler)

        # begin update loop
        while not rospy.is_shutdown():

            # process all queue measurements and ci messages
            self.update()

            # publish state estimate
            self.publish_estimate()

            # sleep until next update
            rate.sleep()

    def control_input_cb(self,msg):
        """
        Saves most recent control input.
        """
        # NOTE: currently faking this with differencing twist commands
        twist_array = [[msg.twist.linear.x],[msg.twist.linear.y]]
        twist_array = np.array(twist_array)
        self.recent_control_input = (twist_array - self.recent_twist)/self.update_rate
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
        if msg.dest == self.agent_id:
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
        if np.trace(self.agent.local_filter.P) > self.tau*self.agent.local_filter.x.shape[0]:

            # generate CI requests
            for conn in self.connections[self.agent_id]:
                # request state of connection and queue
                rospy.logdebug('[Agent {}]: waiting for CI update service from agent_{} to become available'.format(self.agent_id,conn))
                rospy.wait_for_service('/agent_{}/ci_update'.format(conn))

                # create service proxy to send request
                ci_request = rospy.ServiceProxy('/agent_{}/ci_update'.format(conn),CIUpdate)
                # generate ros state message
                request_msg = python2ros_state(self.agent.gen_ci_message(conn,self.neighbor_connections[conn]))
                
                # send request and wait for response
                try:
                    res = ci_request(request_msg)
                    rospy.logdebug('[Agent {}]: CI update response received'.format(self.agent_id))
                    # add response to ci queue
                    self.ci_queue.put(deepcopy(res.response_state))
                except rospy.ServiceException as e:
                    rospy.logerr('[Agent {}]: CI update service request failed: {}'.format(self.agent_id,e))

    def process_ci_queue(self):
        """
        Process queued CI update messages, and perform CI and conditional updates for each.
        """
        rospy.logdebug('[Agent {}]: Emptying CI message queue...'.format(self.agent_id))

        # get current number of messages in queue
        num_messages = self.ci_queue.qsize()

        # grab above number of messages from queue
        ci_messages = [self.ci_queue.get() for x in range(num_messages)]

        rospy.logdebug('[Agent {}]: Grabbed {} message(s) from CI message queue.'.format(self.agent_id,num_messages))

        return ci_messages

    def ci_service_handler(self,req):
        """
        Handler for CI service requests.
        """
        rospy.logdebug('[Agent {}]: CI update request received'.format(self.agent_id))
        # queue recevied request message
        self.ci_queue.put(deepcopy(req.request_state))

        # generate ci message for response
        res_msg = self.agent.gen_ci_message(req.request_state.src,
                    self.neighbor_connections[req.request_state.src])

        # convert to a ros message
        res_msg_ros = python2ros_state(res_msg)
        
        # get state as response
        return res_msg_ros

    def get_state_ros(self,dest=None):
        """
        Generates AgentState message for use in ROS network.

        Inputs:

            none

        Returns:

            msg -- generated AgentState message
        """
        # create state message
        msg = AgentState()
        msg.header.stamp = rospy.Time.now()
        msg.src = self.agent_id
        msg.src_meas_connections = self.meas_connections
        msg.src_connections = self.agent_connections
        msg.mean = self.agent.local_filter.x.transpose().tolist()[0]
        msg.covariance = deflate_covariance(self.agent.local_filter.P)
        msg.src_ci_rate = self.agent.ci_trigger_rate

        if dest is not None:
            msg.dest = dest

        return msg

    def publish_estimate(self):
        """
        Publish local state estimate to ROS network.

        Inputs:

            none

        Returns:

            none
        """
        # get ros state message
        msg = self.get_state_ros()        

        # publish message
        self.local_est_pub.publish(msg)

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
        threshold_measurements = self.agent.process_local_measurements(self.recent_control_input,local_measurements_python)

        # convert messages back to ros type
        threshold_measurements_ros = python2ros_measurement(threshold_measurements)

        # published thresholded measurements to connections through comms module
        for msg in threshold_measurements_ros:
            self.comms_meas_pub.publish(msg)

        # process received measurements
        self.agent.process_received_measurements(received_measurements_python)

        # check for CI updates
        self.local_ci()
        ci_messages_ros = self.process_ci_queue()

        # NOTE: THIS IS A HUGE HACK!!!!!!!
        # For reasons unknown, the src_connections field of a message pulled
        # from the CI queue is sometimes a str type hex representation of the
        # connection value:
        #   i.e. for actual value [1], src_connections will be '\x01'
        # This HACK just catches that and manually changes it using ord,
        # but NO IDEA WHY THIS IS HAPPENING IN THE FIRST PLACE!
        # It's probably a race condition in the CI queue...
        for m in ci_messages_ros:
            if type(m.src_connections) is str:
                m.src_connections = [ord(m.src_connections)]
        
            assert((type(m.src_connections) is list) or ((type(m.src_connections) is int) or (type(m.src_connections) is tuple)) )

        # convert ROS CI messages to Python objects
        ci_messages_python = ros2python_state(ci_messages_ros)

        # perform CI and conditional updates
        self.agent.process_ci_messages(ci_messages_python)

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

        agent_id = deepcopy(self.agent_id)
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
        self.agent_connections = connections_new

        est_state_length = len(ids)

        # find the connections, and therefore intersecting states of all connections, used for similarity transforms in CI updates
        self.neighbor_connections = {}
        self.neighbor_meas_connections = {}
        # loop through all connections of each of self's connections
        for i in self.connections[self.agent_id]:
            if i is not self.agent_id:
                neighbor_agent_id = deepcopy(i)
                ids_new = sorted(deepcopy(self.connections[neighbor_agent_id]))
                ids_new.append(neighbor_agent_id)

                # build list of distance one and distance two neighbors for each agent
                # each agent gets full list of connections
                neighbor_conn_ids = []
                for j in range(0,len(self.connections[neighbor_agent_id])):
                    for k in range(0,len(self.connections[self.connections[neighbor_agent_id][j]])):
                        # if not any(self.connections[self.connections[neighbor_agent_id][j]][k] == x for x in neighbor_conn_ids):
                            # neighbor_conn_ids += deepcopy(self.connections[self.connections[neighbor_agent_id][j]])
                        if not self.connections[self.connections[neighbor_agent_id][j]][k] in neighbor_conn_ids:
                            neighbor_conn_ids += deepcopy(self.connections[self.connections[neighbor_agent_id][j]])

                        # remove agent's own id from list of neighbors
                        if neighbor_agent_id in neighbor_conn_ids:
                            neighbor_conn_ids.remove(neighbor_agent_id)

                # combine with direct connection ids and sort
                ids_new = list(set(sorted(ids_new + neighbor_conn_ids)))

                # divide out direct measurement connections and all connections
                neighbor_connections_new = list(set(sorted(neighbor_conn_ids + deepcopy(self.connections[neighbor_agent_id]))))
                meas_connections_new = deepcopy(self.connections[neighbor_agent_id])
                self.neighbor_meas_connections[i] = deepcopy(meas_connections_new)
                self.neighbor_connections[i] = deepcopy(neighbor_connections_new)

        # construct local estimate
        # TODO: remove hardcoded 4
        n = (est_state_length)*4
        F,G,Q = globals()[dynamics_fxn](self.update_rate,est_state_length)

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