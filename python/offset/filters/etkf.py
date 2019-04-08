#!/usr/bin/env python

__author__ = "Ian Loefgren"
__date__ = "4.1.2019"

"""
Implements the event-triggered linear kalman filter with implicit measurement
fusion.

Usage:
"""

import numpy as np
from scipy.stats import norm
from copy import deepcopy
import pudb

class ETKF(object):

    def __init__(self,F,G,H,M,Q,R_abs,R_rel,x0,P0,delta,agent_id,
                    connections,meas_connection):
        self.F = F
        self.G = G
        self.H = H
        self.M = M
        self.Q = Q
        self.R_abs = R_abs
        self.R_rel = R_rel
        self.x = x0
        self.P = P0
        self.delta = delta
        self.msg_sent = 0
        self.total_msg = 0
        self.agent_id = agent_id
        self.connection = connections
        self.meas_connection = meas_connection
        self.state_history = [x0]
        self.cov_history = [P0]

    def get_location(self,id_):
        """
        Get location of agent specified by :param id in state estimate.

        :param id - scalar id

        TODO:
        - add support for list of ids
        """
        loc = []

        # create list of agents in state estimate, incl. self
        ids = deepcopy(self.connection)
        ids.append(self.agent_id)
        ids.sort()

        # find location of id in state estimate ids, as well as indices
        loc = ids.index(id_)

        return loc

    def get_id(self,loc):
        """ 
        Get id of agent from location in state estimate
        
        :param loc -> int - location in state estimate
        """
        ids = self.connection
        ids.append(self.agent_id)
        ordered_ids = ids.sort()
        return ordered_ids[loc]

    def predict(self,u):
        """
        Propagates current estimate according to dynamics.

        Inputs:
        
            u -- control input vector for all agents in estimate. This will be
                    zeros for all rows except those that corresponse to the agent
                    who is running this filter

        Returns:

            None
        """

        x_curr = np.dot(self.F,self.x) + np.dot(self.G,u)
        P_curr = np.dot(self.F,np.dot(self.P,self.F.transpose())) + self.Q

        self.x = x_curr
        self.xpred = x_curr
        self.P = P_curr

        return x_curr, P_curr

    def threshold(self,msg):
        """
        Thresholds measurement components against curret estimate, and creates
        boolean vector of whether each element of that measurement should be
        transmitted or not.

        Inputs:

            msg -- measurement msg

        Returns:

            src             -- id of agent that sent message (this agent in this function)
            dest            -- id of agent message is intended for
            target          -- id of agent that is target of relative measurement
            outgoing_status -- list of send statuses for each measurement element
            type_           -- type of measurement
            outgoing_data   -- actual measurement data to be transmitted
        """

        # unpack message
        src = msg.src
        dest = msg.dest
        target = msg.target
        status = msg.status
        type_ = msg.type_
        data = msg.data

        # get locations in state estimate of measurement src and target
        src_loc = self.get_location(src)
        target_loc = self.get_location(target)

        # initialize new status and data arrays
        outgoing_status = []
        outgoing_data = []

        # threshold each measurement element
        for i in range(0,len(data)):

            # create measurement fxn
            #TODO this needs to be generalized to measurements are any length
            # as well as of any type
            H = np.zeros( (1,self.F.shape[0]) )
            if type_ == "abs":
                H[0,4*src_loc+2*i] = 1
            elif type_ == "rel":
                H[0,4*src_loc+2*i] = 1
                H[0,4*target_loc+2*i] = -1

            # predicted measurement by filter
            meas_pred = np.dot(H,self.x)
            # innovation
            innov = data[i] - meas_pred

            # threshold
            if np.abs(innov) > self.delta:
                outgoing_status.append(1)
                outgoing_data.append(data[i])
            else:
                outgoing_status.append(0)

        return src, dest, target, outgoing_status, type_, outgoing_data

    def explicit_update(self,src_loc,target_loc,type_,meas_val,data_idx):
        """
        Kalman filter update with measurement data.

        Inputs:

            src_loc         -- location in state estimate of agent that sent msg
            target_loc      -- location in state estimate of target of relative
                                measurement
            type_           -- measurement type
            meas_val        -- measurement element data
            data_idx        -- measurement data location in measurement

        Returns:

            x_curr -- updated state estimate
            P_curr -- updated estimate covariance
        """

        i = data_idx
        # create measurement fxn
        #TODO this needs to be generalized to measurements are any length
        # as well as of any type
        H = np.zeros( (1,self.F.shape[0]) )
        if type_ == "abs":
            H[0,4*src_loc+2*i] = 1
            R = self.R_abs[0,0]
        elif type_ == "rel":
            H[0,4*src_loc+2*i] = 1
            H[0,4*target_loc+2*i] = -1
            R = self.R_rel[0,0]

        # compute predicted measurement and innovation
        meas_pred = np.dot(H,self.x)
        innov = meas_val - meas_pred

        # compute Kalman gain
        K = np.dot(np.dot(self.P,H.transpose()),np.linalg.inv(np.dot(np.dot(H,self.P),
                H.transpose()) + R))

        # update state
        x_curr = self.x + np.dot(K,innov)
        # update covariance
        P_curr = np.dot((np.eye(self.P.shape[0]) - np.dot(K,H)),self.P)

        self.x = x_curr
        self.P = 0.5*P_curr + 0.5*P_curr.transpose()
        # self.state_history.append(x_curr)
        # self.cov_history.append(P_curr)

        return x_curr,P_curr

    def implicit_update(self,src_loc,target_loc,type_,x_local,P_local,data_idx):
        """
        Implicit, set-valued measurement update.

        Inputs:

            src_loc         -- location in state estimate of agent that sent msg
            target_loc      -- location in state estimate of target of relative
                                measurement
            type_           -- measurement type
            x_local         -- snapshot of estimate before measurement update
            P_local         -- snapshot of estimate covariance before meas update
            data_idx        -- measurement data location in measurement

        Returns:

            x_curr -- updated state estimate
            P_curr -- updated estimate covariance
        """

        # phi = lambda z: (1/np.sqrt(2*np.pi))*np.exp(-0.5*(z**2))
        # fxn handles for standard normal distribution pdf and cdf
        phi = norm.pdf
        Qfxn = norm.cdf

        i = data_idx
        # create measurement fxn
        #TODO this needs to be generalized to measurements are any length
        # as well as of any type
        H = np.zeros( (1,self.F.shape[0]) )
        if type_ == "abs":
            H[0,4*src_loc+2*i] = 1
            R = self.R_abs[0,0]
        elif type_ == "rel":
            H[0,4*src_loc+2*i] = 1
            H[0,4*target_loc+2*i] = -1
            R = self.R_rel[0,0]

        #
        mu = np.dot(H,self.x) - np.dot(H,self.xpred)

        #
        Qe = np.dot(H,np.dot(P_local,H.transpose())) + R

        # 
        a = np.dot(H,x_local) - np.dot(H,self.xpred)

        arg1 = (-self.delta + a - mu)/np.sqrt(Qe)
        arg2 = (self.delta + a - mu)/np.sqrt(Qe)

        zbar = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2)))*np.sqrt(Qe)
        dcal = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2))**2) - ((arg1)*phi(arg1)-arg2*phi(arg2)/(Qfxn(arg1)-Qfxn(arg2)))

        # compute Kalman gain
        K = np.dot(np.dot(self.P,H.transpose()),np.linalg.inv(np.dot(np.dot(H,self.P),
                H.transpose()) + R))

        x_curr = self.x + np.dot(K,zbar)

        # P = P - dcal * P * H^T * inv(H * P * H^T + R) * H * P
        invquant = np.dot(H,np.dot(self.P,H.transpose())) + R
        P_curr = self.P - np.multiply(dcal,np.dot(self.P,np.dot(
            H.transpose(),np.dot(np.linalg.inv(invquant),np.dot(H,self.P)))))

        # update filter values
        self.x = x_curr
        self.P = 0.5*P_curr + 0.5*P_curr.transpose()

        return x_curr, P_curr

    def msg_update(self,msg,x_local=None,P_local=None):
        """
        Update function to handle measurement messages.

        Inputs:

            msg     -- measurement message
            x_local -- snapshot of estimate before measurement update
            P_local -- snapshot of estimate covariance before meas update

            Note: x_local and P_local are optional -- if not provided, a local,
                    explicit update will be performed for all elements

        Returns:

            none
        """

        # unpack msg
        src = msg.src
        dest = msg.dest
        target = msg.target
        status = msg.status
        type_ = msg.type_
        data = msg.data
        
        # get locations in state estimate of measurement src and target
        src_loc = self.get_location(src)
        target_loc = self.get_location(target)

        # fuse measurement elements one by one
        data_cnt = 0
        for i in range(0,len(status)):

            # if status for element is true, fuse explicitly
            if status[i]:
                prev_shape = self.x.shape
                self.explicit_update(src_loc,target_loc,type_,data[data_cnt],i)
                assert(self.x.shape == prev_shape)
                data_cnt += 1
            # else, implcit update
            else:
                prev_shape = self.x.shape
                self.implicit_update(src_loc,target_loc,type_,x_local,P_local,i)
                assert(self.x.shape == prev_shape)

def test_etkf():
    pass

if __name__ == "__main__":
    test_etkf()