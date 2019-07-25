#!/usr/bin/env python

from __future__ import division

__author__ = "Ian Loefgren"
__date__ = "5.21.2019"

"""
Implements the event-triggered extended kalman filter with implicit measurement
fusion.

Usage:
"""

import numpy as np
from scipy.stats import norm
from scipy.special import erf
from scipy.linalg import block_diag
from copy import deepcopy
import pudb
# pudb.set_trace()

class ETEKF(object):

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
        ids = list(self.connection)
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
        ids = list(self.connection)
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
            # H = np.zeros( (1,self.F.shape[0]) )
            # if type_ == "abs":
            #     H[0,4*src_loc+2*i] = 1
            # elif type_ == "rel":
            #     H[0,4*src_loc+2*i] = 1
            #     H[0,4*target_loc+2*i] = -1
            H = np.zeros( (1,self.x.shape[0]) )
            if type_ == "abs":
                # H[0,6*src_loc+3*i] = gps_jacobian(self.x,row=i)
                # R = self.R_abs[data_idx,data_idx]
                h = gps(self.x[6*src_loc:6*src_loc+6],row=i)
            elif type_ == "rel":
                # H_tmp = range_az_el_jacobian(self.x,self.x,row=i)
                # H[0,6*src_loc+3*i] = H_tmp[0:6]
                # H[0,6*target_loc+3*i] = H_tmp[6:12]
                # R = self.R_rel[data_idx,data_idx]
                h = range_az_el(self.x[6*src_loc:6*src_loc+6],self.x[6*target_loc:6*target_loc+6],row=i)

            # predicted measurement by filter
            # meas_pred = np.dot(H,self.x)
            meas_pred = h
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
        # H = np.zeros( (1,self.F.shape[0]) )
        # if type_ == "abs":
        #     H[0,4*src_loc+2*i] = 1
        #     R = self.R_abs[0,0]
        # elif type_ == "rel":
        #     H[0,4*src_loc+2*i] = 1
        #     H[0,4*target_loc+2*i] = -1
        #     R = self.R_rel[0,0]
        H = np.zeros( (1,self.F.shape[0]) )
        if type_ == "abs":
            h = gps(self.x[6*src_loc:6*src_loc+6],row=data_idx)
            H[0,6*src_loc:6*src_loc+6] = gps_jacobian(self.x,row=data_idx)
            R = self.R_abs[data_idx,data_idx]
        elif type_ == "rel":
            h = range_az_el(self.x[6*src_loc:6*src_loc+6],self.x[6*target_loc:6*target_loc+6],row=data_idx)
            H_tmp = range_az_el_jacobian(self.x[6*src_loc:6*src_loc+6],self.x[6*target_loc:6*target_loc+6],row=data_idx)
            H[0,6*src_loc:6*src_loc+6] = H_tmp[0:6]
            H[0,6*target_loc:6*target_loc+6] = H_tmp[6:12]
            R = self.R_rel[data_idx,data_idx]

        # compute predicted measurement and innovation
        # meas_pred = np.dot(H,self.x)
        meas_pred = h
        innov = meas_val - meas_pred
        innov = np.expand_dims(innov,axis=1)

        # compute Kalman gain
        K = np.dot(np.dot(self.P,H.transpose()),np.linalg.inv(np.dot(np.dot(H,self.P),
                H.transpose()) + R))

        assert(K.shape == (12,1))

        # update state
        x_curr = self.x + np.dot(K,innov)
        assert(x_curr.shape == (12,1))

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

        # fxn handles for standard normal distribution pdf and cdf
        # phi = norm.pdf
        phi = lambda z: (1/np.sqrt(2*np.pi))*np.exp(-0.5*(z**2)) 
        # Qfxn = norm.cdf
        Qfxn = lambda x: 1 - 0.5*(1+erf(x/np.sqrt(2)))

        i = data_idx
        # create measurement fxn
        #TODO this needs to be generalized to measurements are any length
        # as well as of any type
        # H = np.zeros( (1,self.F.shape[0]) )
        # if type_ == "abs":
        #     H[0,4*src_loc+2*i] = 1
        #     R = self.R_abs[0,0]
        # elif type_ == "rel":
        #     H[0,4*src_loc+2*i] = 1
        #     H[0,4*target_loc+2*i] = -1
        #     R = self.R_rel[0,0]
        H = np.zeros( (1,self.x.shape[0]) )
        if type_ == "abs":
            # compute predicted measurements
            h = gps(self.x[6*src_loc:6*src_loc+6],row=data_idx)
            h_xpred = gps(self.xpred[6*src_loc:6*src_loc+6],row=data_idx)
            h_xlocal = gps(x_local[6*src_loc:6*src_loc+6],row=data_idx)

            H[0,6*src_loc+3*i] = gps_jacobian(self.x,row=data_idx)
            R = self.R_abs[data_idx,data_idx]
        elif type_ == "rel":
            # compute predicted measurements
            h = range_az_el(self.x[6*src_loc:6*src_loc+6],self.x[6*target_loc:6*target_loc+6],row=data_idx)
            h_xpred = range_az_el(self.xpred[6*src_loc:6*src_loc+6],self.xpred[6*target_loc:6*target_loc+6],row=data_idx)
            h_xlocal = range_az_el(x_local[6*src_loc:6*src_loc+6],x_local[6*target_loc:6*target_loc+6],row=data_idx)
            
            H_tmp = range_az_el_jacobian(self.x,self.x,row=data_idx)
            H[0,6*src_loc+3*i] = H_tmp[0:6]
            H[0,6*target_loc+3*i] = H_tmp[6:12]
            R = self.R_rel[data_idx,data_idx]

        #
        # mu = np.dot(H,self.x) - np.dot(H,self.xpred)
        mu = h - h_xpred

        #
        Qe = np.dot(H,np.dot(P_local,H.transpose())) + R

        # 
        # a = np.dot(H,x_local) - np.dot(H,self.xpred)
        a = h_xlocal - h_xpred

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

def range_az_el(state1,state2,row=-1):
    """
    Range-azimuth-elevation measurement function. Used for relative
    measurements between vehicles.

    Inputs:

        state1 -- state vector of vehicle 1
        state2 -- state vector of vehicle 2
        row -- index of row to compute, used for per element computations (default=-1: all rows)

        state vector is assumed to be of the form:
            [x xdot y ydot z zdot]

    Returns:

        meas -- measurement vector: [range [m], azimuth [rad], elevation [rad]]
    """
    # assign state variables
    x1 = state1[0]
    y1 = state1[2]
    z1 = state1[4]
    x2 = state2[0]
    y2 = state2[2]
    z2 = state2[4]

    meas = np.array( [0, 0, 0] )

    # compute range
    meas[0] = np.sqrt( (x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2 )
    # compute elevation
    meas[1] = np.arctan2((z1-z2), (np.sqrt( (x1-x2)**2 + (y1-y2)**2 )))
    # compute azimuth
    meas[2] = np.arctan2( (y1-y2), (x1-x2) )

    meas = np.expand_dims(meas,axis=1)
    assert(meas.shape == (3,1))

    if row == -1:
        return meas
    else:
        return meas[row]

def range_az_el_jacobian(state1,state2,row=-1):
    """
    Computes Jacobian for the range-azimuth-elevation measurement function.

    Inputs:

        state1 -- state vector of vehicle 1
        state2 -- state vector of vehicle 2
        row -- index of row to compute, used for per element computations (default=-1: all rows)

        state vector is assumed to be of the form:
            [x xdot y ydot z zdot]

    Returns:

        rae_jacobian -- jacobian for measurement fxn given above state [3x12 numpy array]
    """
    # assign state variables
    x1 = state1[0]
    y1 = state1[2]
    z1 = state1[4]
    x2 = state2[0]
    y2 = state2[2]
    z2 = state2[4]

    # create anonymous fxns for repeated fxns
    range_ = lambda x1, x2, y1, y2, z1, z2: (x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2
    el = lambda x1, x2, y1, y2, z1, z2: (z1-z2)/(np.sqrt( (x1-x2)**2 + (y1-y2)**2 ))
    az = lambda x1, x2, y1, y2: (y1-y2)/(x1-x2)

    rae_jacobian = np.zeros((3,12))

    # jacobian row 1: range measurement derivatives
    rae_jacobian[0,0] = range_(x1,x2,y1,y2,z1,z2)**(-0.5) * (x1-x2)
    rae_jacobian[0,2] = range_(x1,x2,y1,y2,z1,z2)**(-0.5) * (y1-y2)
    rae_jacobian[0,4] = range_(x1,x2,y1,y2,z1,z2)**(-0.5) * (z1-z2)
    rae_jacobian[0,6] = range_(x1,x2,y1,y2,z1,z2)**(-0.5) * -(x1-x2)
    rae_jacobian[0,8] = range_(x1,x2,y1,y2,z1,z2)**(-0.5) * -(y1-y2)
    rae_jacobian[0,10] = range_(x1,x2,y1,y2,z1,z2)**(-0.5) * -(z1-z2)

    # jacobian row 2: elevation measurement derivatives
    rae_jacobian[1,0] = (-1/(1+el(x1,x2,y1,y2,z1,z2)**2))*(z1-z2)*( (x1-x2)**2 + (y1-y2)**2 )**(-1.5) * (x1-x2)
    rae_jacobian[1,2] = (-1/(1+el(x1,x2,y1,y2,z1,z2)**2))*(z1-z2)*( (x1-x2)**2 + (y1-y2)**2 )**(-1.5) * (y1-y2)
    # rae_jacobian[1,4] = (-1/(1+el(x1,x2,y1,y2,z1,z2)**2))*(z1-z2)*( (x1-x2)**2 + (y1-y2)**2 )**(-1.5) * (z1-z2)
    rae_jacobian[1,4] = (1/(1+el(x1,x2,y1,y2,z1,z2)**2))*(1/(np.sqrt((x1-x2)**2 + (y1-y2)**2)))
    rae_jacobian[1,6] = (1/(1+el(x1,x2,y1,y2,z1,z2)**2))*(z1-z2)*( (x1-x2)**2 + (y1-y2)**2 )**(-1.5) * (x1-x2)
    rae_jacobian[1,8] = (1/(1+el(x1,x2,y1,y2,z1,z2)**2))*(z1-z2)*( (x1-x2)**2 + (y1-y2)**2 )**(-1.5) * (y1-y2)
    # rae_jacobian[1,10] = (1/(1+el(x1,x2,y1,y2,z1,z2)**2))*(z1-z2)*( (x1-x2)**2 + (y1-y2)**2 )**(-1.5) * (z1-z2)
    rae_jacobian[1,10] = -(1/(1+el(x1,x2,y1,y2,z1,z2)**2))*(1/(np.sqrt((x1-x2)**2 + (y1-y2)**2)))

    # jacobian row 3: azimuth measurement derivatives
    # rae_jacobian[2,0] = (-1/(1+az(x1,x2,y1,y2)**2))*(y1-y2)*(x1-x2)**(-2)
    rae_jacobian[2,0] = -(y1-y2)/( (x1-x2)**2 + (y1-y2)**2 )
    # rae_jacobian[2,2] = (1/(1+az(x1,x2,y1,y2)**2))*( (y1-y2)/(x1-x2) )
    rae_jacobian[2,2] = (x1-x2)/( (x1-x2)**2 + (y1-y2)**2 )
    rae_jacobian[2,4] = 0
    # rae_jacobian[2,6] = (1/(1+az(x1,x2,y1,y2)**2))*(y1-y2)*(x1-x2)**(-2)
    rae_jacobian[2,6] = (y1-y2)/( (x1-x2)**2 + (y1-y2)**2 )
    # rae_jacobian[2,8] = (-1/(1+az(x1,x2,y1,y2)**2))*( (y1-y2)/(x1-x2) )
    rae_jacobian[2,8] = -(x1-x2)/( (x1-x2)**2 + (y1-y2)**2 )
    rae_jacobian[2,10] = 0

    if row == -1:
        row = np.arange(0,3)

    return rae_jacobian[row,:]

def gps(state,row=-1):
    """
    Absolute position psuedo GPS measurement function.

    Inputs:

        state -- state vector
        row -- index of row to compute, used for per element computations (default=-1: all rows)

        state vector is assumed to be of the form:
            [x xdot y ydot z zdot]

    Returns:

        meas -- measurement vector: [x y z] (ENU coords)
    """
    meas = np.array( [state[0],state[2],state[4]] )
    
    assert(meas.shape == (3,1))

    if row == -1:
        return meas
    else:
        return meas[row]

def gps_jacobian(state,row=-1):
    """
    Absolute position psuedo GPS measurement function.

    Inputs:

        state -- state vector
        row -- index of row to compute, used for per element computations (default=-1: all rows)

        state vector is assumed to be of the form:
            [x xdot y ydot z zdot]

    Returns:

        gps_jacobian -- measurement function jacobian for above state [3x6 numpy array]
    """

    gps_jacobian = np.zeros((3,6))

    gps_jacobian[0,0] = 1
    gps_jacobian[1,2] = 1
    gps_jacobian[2,4] = 1
    
    assert(gps_jacobian.shape == (3,6))

    if row == -1:
        row = np.arange(0,3)

    return gps_jacobian[row]


def test_etekf():
    """
    ET-EKF class test scenario, w/ NCV dynamics, and range-az-el sensors to landmark.
    """
    from offset.dynamics import lin_ncv

    # seed random number generator for predictable results
    np.random.seed(100)

    # define one agent modeled by dubin's unicycle
    dt = 0.1
    tfin = 30

    # define initial estimate and covariance, and constant input (speed, and turning vel)
    # x0 = np.array( [ [0], [0], [0], [0], [10], [0] ] ,ndmin=2)
    x0 = np.array([0,0,0,0,10,0,0,0,0,0,0,0])
    x0 = np.expand_dims(x0,axis=1)
    P0 = 1000*np.eye(6)
    P0 = block_diag(P0,np.eye(6))
    # u = np.array( [ [3], [0.2] ], ndmin=2)

    # F_local, G_local, _ = globals()['lin_ncv'](dt,1)

    F_local = np.array(((1,dt,0,0,0,0),
                (0,1,0,0,0,0),
                (0,0,1,dt,0,0),
                (0,0,0,1,0,0),
                (0,0,0,0,1,dt),
                (0,0,0,0,0,1)) )

    G_local = np.array(((0.5*(dt**2),0),
                (dt,0),
                (0,0.5*(dt**2)),
                (0,dt),
                (0,0),
                (0,0)) )

    Q_local_true = np.array( ((0.0003,0.005,0,0,0,0),
                            (0.005,0.1,0,0,0,0),
                            (0,0,0.0003,0.005,0,0),
                            (0,0,0.005,0.1,0,0),
                            (0,0,0,0,0.0003,0.005),
                            (0,0,0,0,0.005,0.1)) )

    R_abs = 10*np.eye(3)
    R_rel = np.array( ((3,0,0),
                        (0,5*(np.pi/180),0),
                        (0,0,5*(np.pi/180))) )


    F = block_diag(F_local,np.eye(6))
    G = np.vstack( (G_local,np.zeros(G_local.shape)) )
    Q = block_diag(Q_local_true,np.zeros(Q_local_true.shape))

    # create EKF filter object
    filt = ETEKF(F, G, 0, 0, Q, R_abs, R_rel, x0, P0, 0.0, 0, [0], [0])

    x_true_history = [x0]

    x_est = [x0]
    x_true = x0

    x_true_x = [x0[0]]
    x_est_x = [x0[0]]

    x_true_y = [x0[2]]
    x_est_y = [x0[2]]

    x_true_z = [x0[4]]
    x_est_z = [x0[4]]

    P_est = [P0]
    meas = []

    # generate truth data and noisy truth data
    for i in np.arange(dt,tfin,dt):
        # generate control input
        u = np.array( ( (2*np.cos(0.75*i)), (2*np.sin(0.75*i)) ) )
        u = np.expand_dims(u,axis=1)
        # generate process noise
        noise = np.random.multivariate_normal([0,0,0,0,0,0,0,0,0,0,0,0],Q).transpose()
        noise = np.expand_dims(noise,axis=1)
        # noise = np.array(noise,ndmin=2).transpose()
        # generate new true state
        x_true = np.dot(F,x_true) + np.dot(G,u) + noise

        x_true_history.append(x_true)

        # generate sensor measurement
        y_true_rel = range_az_el(x_true[:6],[0,0,0,0,0,0],row=-1)
        y_true_abs = gps(x_true[:6],row=-1)
        # print(y_true)
        assert(y_true_rel.shape == (3,1))
        assert(y_true_abs.shape == (3,1))
        # print(y_true)
        # generate measurement noise
        meas_noise_rel = np.random.multivariate_normal([0,0,0],R_rel).transpose()
        meas_noise_rel = np.expand_dims(meas_noise_rel,axis=1)

        meas_noise_abs = np.random.multivariate_normal([0,0,0],R_abs).transpose()
        meas_noise_abs = np.expand_dims(meas_noise_abs,axis=1)
        # print(meas_noise)
        # meas_noise = np.array(meas_noise,ndmin=2).transpose()
        # add noise
        y_meas_rel = y_true_rel + meas_noise_rel
        y_meas_abs = y_true_abs + meas_noise_abs

        assert(y_meas_rel.shape == (3,1))
        assert(y_meas_abs.shape == (3,1))

        # update filter
        # propagate
        filt.predict(u)

        # measurement update (per element)
        for j in range(0,y_meas_rel.shape[0]):
            # print(y_meas[j])
            filt.explicit_update(0,1,"rel",y_meas_rel[j],j)
        for j in range(0,y_meas_abs.shape[0]):
            # print(y_meas[j])
            filt.explicit_update(0,1,"abs",y_meas_abs[j],j)

        x_curr = deepcopy(filt.x)
        P_curr = deepcopy(filt.P)

        # save estimate
        x_est.append(x_curr)
        P_est.append(P_curr)
        x_est_x.append(x_curr[0,0])
        x_est_y.append(x_curr[2,0])
        x_est_z.append(x_curr[4,0])

        x_true_x.append(x_true[0,0])
        x_true_y.append(x_true[2,0])
        x_true_z.append(x_true[4,0])


    # soln1wnoise = soln1.y[0:3].transpose() + np.random.multivariate_normal([0,0,0],Q,int((tfin/dt)+1))


    

    # (self,F,G,H,M,Q,R_abs,R_rel,x0,P0,delta,agent_id,
                    # connections,meas_connection):


    

    # input_tvec = np.linspace(0,tfin,num=(tfin/dt)+1)
    # for i in range(0,len(input_tvec)):
    #     # propagate filter
    #     filt.predict(input_tvec[i],u);
        
    #     # update filter
    #     # print([soln1.y[0][i],soln1.y[1][i],soln1.y[2][i]])
    #     y = range_bearing([soln1.y[0][i],soln1.y[1][i],soln1.y[2][i]],R)
    #     meas.append(y)
    #     x_curr, P_curr = filt.update(input_tvec[i],y);
        
    #     # save estimate
    #     x_est.append(x_curr)
    #     P_est.append(P_curr)
    #     x_est_x.append(x_curr[0])
    #     x_est_y.append(x_curr[1])
    #     x_est_theta.append(x_curr[2])

    import matplotlib.pyplot as plt

    # configure pyplot for using latex
    plt.rc('text', usetex=True)
    plt.rc('font',family='serif')

    dim = 0
    idx = [0,1,2,3,4,5]
    time_vec = np.arange(0,tfin,dt)

    # est_data_vec = np.concatenate([np.array(x[idx]) for x in x_est],axis=0)
    # truth_data_vec = np.concatenate([np.expand_dims(x,axis=1) for x in x_true_history],axis=1)
    # var_data_vec = np.concatenate([np.expand_dims(np.diag(x[np.ix_(idx,idx)]),axis=1) for x in P_est],axis=1)

    plt.figure(1)
    plt.grid(True)
    
    plt.plot(x_true_x,x_true_y)
    plt.plot(x_est_x,x_est_y)
    
    plt.legend(['true position','etekf estimate'],loc='lower right')

    plt.figure(2)

    plt.grid()

    plt.plot(time_vec,np.array(x_est_x) - np.array(x_true_x))
    plt.plot(time_vec,np.array(x_est_y) - np.array(x_true_y))
    # plt.plot(time_vec,(est_data_vec[dim,:]-truth_data_vec[dim,:]),'r')
    # plt.plot(time_vec,2*np.sqrt(var_data_vec[dim,:]),'r--')
    # plt.plot(time_vec,-2*np.sqrt(var_data_vec[dim,:]),'r--')

    # plt.plot(time_vec,(bl_est_data_vec[dim,:]-truth_data_vec[dim,:]),'g')
    # plt.plot(time_vec,2*np.sqrt(bl_var_data_vec[dim,:]),'g--')
    # plt.plot(time_vec,-2*np.sqrt(bl_var_data_vec[dim,:]),'g--')

    plt.xlabel('Time [s]')
    plt.ylabel('Est error [m]')
    # plt.title(r'Agent {} ownship $\xi$ est. err: $\delta={}$, $\tau_g={}$, msg drop={}'.format(id_+1,metadata['delta'],metadata['tau_goal'],metadata['msg_drop_prob']))
    plt.legend(['Est error',r'$\pm 2\sigma$',''])

    plt.show()

if __name__ == "__main__":
    test_etekf()