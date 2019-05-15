#!/usr/bin/env python

"""
Communications module for Minau AUVs. Handles message passing, and simulates
a communications stack, including potential dropping of messages.
"""

import rospy
import numpy as np

from cohrint_minau.msg import AgentMeasurement, AgentState

class CommsModule(object):

    def __init__(self):
        
        # initialize ros node
        rospy.init_node('comms_module')

        # load parameters from the parameter server
        self.agent_name = rospy.get_namespace()
        self.drop_incoming_prob = rospy.get_param('comm_drop_prob')
        self.connections = rospy.get_param('connections')

        # create topic subscriptions for incoming messages (i.e | connections -> comms |-> agent)
        for sub_name in self.connections:
            rospy.Subscriber('/' + sub_name + '/comms_meas', AgentMeasurement, self.incoming_message)
            rospy.Subscriber('/' + sub_name + '/comms_state', AgentState, self.incoming_message)

        # create topic subscriptions for outgoing messages (i.e. | agent -> comms | -> connections)
        rospy.Subscriber('agent_meas_outgoing', AgentMeasurement, self.outgoing_message)
        rospy.Subscriber('agent_state_outgoing', AgentState, self.outgoing_message)

        # create publishers for outgoing topics (i.e agent -> | comms -> connections |)
        self.comms_meas_pub = rospy.Publisher('comms_meas', AgentMeasurement, queue_size=10)
        self.comms_state_pub = rospy.Publisher('comms_state', AgentState, queue_size=10)

        # create publishers for incoming messages (i.e connections -> | comms -> agent | )
        self.agent_meas_pub = rospy.Publisher('agent_meas_incoming', AgentMeasurement, queue_size=10)
        self.agent_state_pub = rospy.Publisher('agent_state_incoming', AgentState, queue_size=10)

        # log info
        rospy.loginfo('Initialized {} comms module'.format(self.agent_name))

        # wait for messages
        rospy.spin()

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
            if msg._type == 'cohrint_minau/AgentMeasurement':
                self.agent_meas_pub.publish(msg)
            elif msg._type == 'cohrint_minau/AgentState':
                self.agent_state_pub.publish(msg)
        

    def outgoing_message(self,msg):
        """
        Process outgoing message, and forward to connection.

        Inputs:

            msg -- measurement or state message to be sent from agent to connections

        Returns:

            none
        """
        if msg._type == 'cohrint_minau/AgentMeasurement':
            self.comms_meas_pub.publish(msg)
        elif msg._type == 'cohrint_minau/AgentState':
            self.comms_state_pub.publish(msg)



if __name__ == "__main__":
    CommsModule()