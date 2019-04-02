#!/usr/bin/env python

__author__ = "Ian Loefgren"
__date__ = "1.4.2019"

"""
Implements framework for handling simple message passing in a distributed
event-triggered filtering scenario.

Usage:
"""

import numpy as np

# from src.filters import ETKF
# from utils import meas_msg

class Agent(object):
    
    def __init__(self,agent_id,connections,meas_connections,neighbor_connections,
                    local_filter,common_estimates,x_true,msg_drop_prob,tau_goal,tau):    
        # agent id
        self.agent_id = agent_id
        
        # ids of connections
        self.connections = connections
        self.meas_connections = meas_connections
        self.neighbor_connections = neighbor_connections
        
        # number of states per platform
        self.num_states = 4
        
        # filter selfect for local estimation
        self.local_filter = local_filter
        
        # struct with ids of connections and filters estimating common
        self.common_estimates = common_estimates
        
        # true starting position
        self.true_state = x_true
        
        # CI trigger count
        self.ci_trigger_cnt = 0
        self.ci_trigger_rate = 0
        
        self.msgs_sent = 0
        self.total_msgs = 0
        
        self.msg_success_prob = 1-msg_drop_prob
        
        # CI thresholding data
        self.tau_goal = tau_goal
        self.tau = tau
        self.connection_tau_rates = np.zeros(len(self.connections),1)
        self.epsilon_1 = 0.05
        self.epsilon_2 = 0.1

    def get_location(self,id_):
        """
        Get location of agent specified by :param id in state estimate, as well as indicies.

        :param id - scalar id

        TODO:
        - add support for list of ids
        """
        loc = []
        idx = []

        # create list of agents in state estimate, incl. self
        ids = self.connections
        ids.append(self.agent_id)
        ordered_ids = sort(ids)

        # find location of id in state estimate ids, as well as indices
        loc = ordered_ids.index(id_)
        idx = list(range(self.num_states*loc,self.num_states*loc+self.num_states))

        return loc, idx

    def get_id(self,loc):
        """ 
        Get id of agent from location in state estimate
        
        :param loc -> int - location in state estimate
        """
        ids = self.connections
        ids.append(self.agent_id)
        ordered_ids = sort(ids)
        return ordered_ids[loc]

    def process_local_measurements(self,input_,local_measurements):
        """
        Handle control input and locally collected measurements.
        Propagates local and common informatin filters, and handles measurement
        messages.

        :param input -> np.array - control input vector
        :param local_measurements -> list of msgs - lisr of meas. msgs

        TODO: compare iterating w/ randperm vs w/o - does it actually matter?
        """
        
        # get self location and index
        agent_loc,agent_idx = self.get_location(self.agent_id)

        # create correctly-sized input vector to apply from local control input
        input_vec = np.zeros( (self.local_filter.G.shape[1],1) )
        input_vec[2*agent_loc:2*agent_loc+1,1] = input_

        # propagate local and common info filters
        self.local_filter.predict(input_vec)
        for filter_ in self.common_estimates:
            # common information filters get no knowledge of control input
            # b/c others have no idea what your control input was
            filter_.predict(np.zeros(filter_.G.shape[1],1))

        # create outgoing and 'loopback' message queues
        outgoing = []
        loopback = []

        # process each collected measurement
        for msg in local_measurements:
            
            # unpack msg
            src = msg.src
            dest = msg.dest
            target = msg.target
            status = msg.status
            type_ = msg.type
            data = msg.data

            # make sure source of measurement is current agent
            assert(src == self.agent_id)

            # update local filter
            self.local_filter.msg_update(msg)

            # threshold measurement and update w/ relevant common info ests
            for id_ in self.meas_connections:

                # update msg destination to be measurement connection
                dest = id_

                # regenerate message
                msg = gen_msg(src,dest,target,status,type_,data)

                # find common estimate associated w/ msg destination
                for filter_ in self.common_estimates:
                    
                    # if filter estimates destination agent 
                    if filter_.meas_connection is msg.dest:

                        # threshold measurement
                        src_,dest_,target_,status_,type_,data_ = filter_.threshold(msg)

                        # TODO: update to call w/ list of ids, not induvidual
                        idx = [agent_idx]
                        loc,idx1 = self.get_location(filter_.connection)
                        idx.append(idx1)
                        idx.sort()

                        # generate new message
                        msg = gen_msg(src_,dest_,target_,status_,type_,data_)

                        # increment msg sent counts
                        self.msgs_sent += sum(status_)
                        self.total_msgs += len(status_)

                        # add msg to update filter to loopback queue and outgoing
                        # (this is so as to not affect thresholding w/ updates)
                        x_local = self.local_filter.x[idx]
                        P_local = self.local_filter.P[idx,idx]

                        outgoing.append(msg)
                        loopback.append([msg,x_local,P_local])

        # process measurements in loopback queue
        for msg in loopback:

            msg_ = msg[0]
            x_local = msg[1]
            P_local = msg[2]

            for filter_ in self.common_estimates:

                # find estimate associated w/ msg destination
                if filter_.meas_connection is msg.dest:

                    # update filter
                    filter_.msg_update(msg,x_local,P_local)

    def process_received_measurements(self,inbox):
        """
        Update local and common information filters w/ inbox msg queue.

        :param inbox -> list - list of recevied messages
        """
        for msg in inbox:

            # make sure message dest is current agent
            assert(msg.dest is self.agent_id)

            # imperfect comms sim: coin flip for dropping message
            # TODO: add binomial draw w/ msgs drop prob here
             
            # update local filter
            x_local = self.local_filter.x
            P_local = self.local_filter.P
            self.local_filter.msg_update(msg,x_local,P_local)

            # update common information filters
            for filter_ in self.common_estimates:

                # TODO: update to call w/ list of ids, not induvidual
                # TODO: filter_.connection might be a vector in matlab code
                idx = [agent_idx]
                loc,idx1 = self.get_location(filter_.connection)
                idx.append(idx1)
                idx.sort()

                x_local_comm = self.local_filter.x[idx]
                P_local_comm = self.local_filter.P[idx,idx]

                # find common estimate associated w/ msg source
                if filter_.meas_connection is msg.src:
                    filter_.msg_update(msg,x_local_comm,P_local_comm)


def test_agent():
    pass

if __name__ == "__main__":
    test_agent()