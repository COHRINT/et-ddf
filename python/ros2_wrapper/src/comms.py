#!/usr/bin/env python

"""
Communications module for vehicles using ET-DDF. Handles message passing, and simulates
a communications stack, including potential dropping of messages.
"""

import os
import sys
import rclpy
import numpy as np

from ament_index_python.packages import get_package_share_directory
print(sys.path)
from etddf.helpers.config_handling import load_config

from etddf_ros2_msgs.msg import AgentMeasurement, AgentState

class CommsModule(object):

    def __init__(self, cfg=None, agent_name=None):
        
        # initialize ros node
        # rospy.init_node('comms_module')
        rclpy.init()
        self.node = rclpy.create_node('comms_module')

        # load parameters from the parameter server
        # self.agent_name = self.node.get_namespace()
        self.agent_name = agent_name
        self.drop_incoming_prob = self.node.get_parameter('comm_drop_prob').value
        self.connections = self.node.get_parameter('meas_connections').value
        print(self.connections)

        # create topic subscriptions for incoming messages (i.e | connections -> comms |-> agent)
        for sub_name in self.connections:
            self.node.create_subscriber(AgentMeasurement, '/' + sub_name + '/comms_meas', self.incoming_message)
            self.node.create_subscriber(AgentState, '/' + sub_name + '/comms_state', self.incoming_message)

        # create topic subscriptions for outgoing messages (i.e. | agent -> comms | -> connections)
        self.node.create_subscriber(AgentMeasurement, 'agent_meas_outgoing', self.outgoing_message)
        self.node.create_subscriber(AgentState, 'agent_state_outgoing', self.outgoing_message)

        # create publishers for outgoing topics (i.e agent -> | comms -> connections |)
        self.comms_meas_pub = self.node.create_publisher(AgentMeasurement, 'comms_meas')
        self.comms_state_pub = self.node.create_publisher(AgentState, 'comms_state')

        # create publishers for incoming messages (i.e connections -> | comms -> agent | )
        self.agent_meas_pub = self.node.create_publisher(AgentMeasurement, 'agent_meas_incoming')
        self.agent_state_pub = self.node.create_publisher(AgentState, 'agent_state_incoming')

        # log info
        self.node.get_logger().info('Initialized {} comms module'.format(self.agent_name))

        # wait for messages
        self.node.spin()

    def incoming_message(self,msg):
        """
        Process incoming message, corrupt or drop it, depending on sim params,
        and forward to associated agent.

        Input:

            msg -- measurement or state message sent from a connection

        Returns: 

            none
        """
        # check if we should drop the message entirely
        if np.random.binomial(1,self.drop_incoming_prob):
            return None
        else:
            if msg._type == 'etddf_ros2_msgs/AgentMeasurement':
                self.agent_meas_pub.publish(msg)
            elif msg._type == 'etddf_ros2_msgs/AgentState':
                self.agent_state_pub.publish(msg)
        

    def outgoing_message(self,msg):
        """
        Process outgoing message, and forward to connection.

        Inputs:

            msg -- measurement or state message to be sent from agent to connections

        Returns:

            none
        """
        if msg._type == 'etddf_ros/AgentMeasurement':
            self.comms_meas_pub.publish(msg)
        elif msg._type == 'etddf_ros/AgentState':
            self.comms_state_pub.publish(msg)

def main():

    # get command line args (or coming from launch file)
    cl_args = sys.argv[1:]
    print(cl_args)

    # load config files (instead of using parameter server)
    # agent_cfg = load_config(get_package_share_directory('etddf_ros2') + '/ros_agent_config.yaml')
    gen_config = load_config(get_package_share_directory('etddf_ros2') + '/points.yaml')

    CommsModule(cfg=gen_config,agent_name=cl_args[0])

if __name__ == "__main__":
    main()