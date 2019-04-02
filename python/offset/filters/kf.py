#!/usr/bin/env python

__author__ = "Ian Loefgren"
__date__ = "1.7.2018"

"""
Linear Kalman filter implementation.
"""

import numpy as np

class KF(object):

    def __init__(self,F,G,H,M,Q,R_abs,R_rel,x0,P0,agent_id):
        self.F = F
        self.G = G
        self.H = H
        self.M = M
        self.Q = Q
        self.R_abs = R_abs
        self.R_rel = R_rel
        self.x = x0
        self.P = P0
        self.state_history = [x0]
        self.cov_history = [P0]
        self.agent_id = agent_id

    def predict(self,u):

        x_curr = np.dot(self.F,self.x) + np.dot(self.G,u)
        P_curr = np.dot(self.F,np.dot(self.P,self.F.transpose())) + self.Q

        self.x = x_curr
        self.P = P_curr

        self.state_history.append(x_curr)
        self.cov_history.append(P_curr)

        return x_curr, P_curr

    def update(self,meas,meas_type,src_id,rel_agent_id):

        for i in range(0,meas.size[0]):
            H = np.zeros([2,self.F.size[0]])
            if meas_type is 'abs':
                H[0,4*(src_id-1)] = 1
                H[1,4*(src_id-1)+2] = 1
                R = self.R_abs
            elif meas_type is 'rel':
                H[0,4*(src_id-1)] = 1
                H[1,4*(src_id-1)+2] = 1
                H[0,4*(rel_agent_id-1)] = -1
                H[1,4*(rel_agent_id-1)+2] = -1
                R = self.R_rel
            else:
                print('Received invalid measurement type {}'.format(meas_type))
                continue

            # extract measurement
            meas_val = meas[:,i:i+1]

            # compute predicted measurement and innovation
            meas_pred = np.dot(H,self.x)
            innov = meas_val - meas_pred

            # compute Kalman gain
            K = np.dot(np.dot(self.P,H.transpose()),np.linalg.inv(np.dot(np.dot(H,self.P),H.transpose()) + R))

            # update state
            x_curr = self.x + np.dot(K,innov)
            # update covariance
            P_curr = np.dot((np.eye(self.P.shape[0]) - np.dot(K,H)),self.P)

            self.x = x_curr
            self.P = P_curr
            self.state_history.append(x_curr)
            self.cov_history.append(P_curr)

            return x_curr,P_curr

def test_kf():
    pass

if __name__ == "__main__":
    test_kf()