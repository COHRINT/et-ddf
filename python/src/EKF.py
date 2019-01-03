#!/usr/bin/env python

__author__ = "Ian Loefgren"
__date__ = "12.28.2018"

"""
Implementation of the extended kalman filter

Usage:
  This implmentation of the extended kalman filter heavily leverages
  Python's ability to create function handles. 
  Call predict() with the time and input to propagate the estimate, then
  update() with the time and measurement to update the estimate.

Inputs:
  - f: nonlinear dynamics function handle
  - h: nonlinear measurement function handle
  - Atilde: CT dynamics Jacobian function handle
  - Gamma: noise array
  - Htilde: CT measurement jacobian function handle
  - dt: timestep
  - x0: state estimate initialization
  - P0: state covariance initilization
"""

import numpy as np
from scipy.linalg import block_diag
from scipy.integrate import solve_ivp

class EKF(object):

    def __init__(self,f,h,Atilde,Gamma,Htilde,Q,R,dt,x0,P0):
        self.f = f;
        self.h = h;
        self.Atilde = Atilde;
        self.Gamma = Gamma;
        self.Htilde = Htilde;
        self.Q = Q;
        self.R = R;
        self.dt = dt;
        self.x = x0;
        self.P = P0;
    
    def predict(self,t,u):
        
        # nonlinear fxn eval for state estimate
        # [~,soln] = ode45(@(t,y) self.f(t,y,0),[t,t+self.dt],[self.x;u]);
        # print(self.x)
        # print(u)
        # print(np.concatenate((self.x,u)).transpose()[0])
        soln = solve_ivp(self.f,[t,t+self.dt],np.concatenate((self.x,u)).transpose()[0],t_eval=[t+self.dt]);
        self.x = soln.y[0:3]
        
        # compute jacobian for current time to compute P approx
        F = np.eye(len(self.x)) + self.dt * self.Atilde(self.x.tolist(),u.transpose().tolist(),t);
        Omega = self.dt * self.Gamma(t);
        self.P = F*self.P*F.transpose() + Omega * self.Q * Omega.transpose();
        
        x_curr = self.x; P_curr = self.P;
    
    def update(self,t,meas):
        
        # nonlinear fxn eval for predicted measurement
        y_pred = self.h(self.x,0,noise=False);
        # print(y_pred)
        
        # compute measurement jacobian and approx kalman gain
        H = self.Htilde(self.x,t)
        K = np.dot(np.dot(self.P,H.transpose()),np.linalg.inv(np.dot(np.dot(H,self.P),H.transpose()) + self.R))
        # print(K)
        
        # update state and covariance estimates
        # print(meas-y_pred.transpose())
        # print(np.dot(K,(meas-y_pred)))
        self.x = self.x + np.dot(K,(meas-y_pred.transpose()).transpose())
        self.P = np.dot((np.eye(self.P.shape[0]) - np.dot(K,H)),self.P)
        
        x_curr = self.x; P_curr = self.P;
        # print(x_curr)

        return x_curr, P_curr

def dubin_uni(t,y):
    x = y[0]
    y_ = y[1]
    theta = y[2]
    v = y[3]
    omega = y[4]
    # dydt = np.array( ((v*np.cos(theta)),(v*np.sin(theta)),(omega),(0),(0)) )
    dydt = [v*np.cos(theta),v*np.sin(theta),omega,0,0]
    return dydt
            

def Atilde(x,u,t):
    return np.array( ((0,0,-u[0][0]*np.sin(x[2][0])), 
                        (0,0,u[0][0]*np.cos(x[2][0])),
                        (0,0,0)) ,dtype='float')

def range_bearing(x_,R,noise=True):
    x = x_[0]
    y = x_[1]
    theta = x_[2]
    y_ = np.array( ((np.arctan2(y,x)),(np.sqrt(x**2 + y**2))) )
    if noise:
        y_ += np.random.multivariate_normal([0,0],R)
    return y_

def Htilde(x,t):
    return np.array( ((-x[1]/(x[0]**2+x[1]**2),-x[0]/(x[0]**2+x[1]**2),0),
                        (x[0]*(x[0]**2+x[1]**2)**(-0.5),x[1]*(x[0]**2+x[1]**2)**(-0.5),0)),dtype='float' )



def test_ekf():
    """
    EKF class test scenario, w/ Dubin's unicycle dynamics, and range and
    bearing sensors to landmark.
    """

    # seed random number generator for predictable results
    np.random.seed(100)

    # define one agent modeled by dubin's unicycle
    dt = 0.1;
    tfin = 30;

    # define initial estimate and covariance, and constant input (speed, and turning vel)
    # x0 = [0 0 0]';
    x0 = np.array( [ [0], [0], [0] ] ,ndmin=2)
    P0 = np.diag([1,1,0.001]);
    u = np.array( [ [3], [0.2] ], ndmin=2);

    Q = np.diag([1,1,0.01]);
    noise = block_diag(Q,[0,0]);
    # [t1,y1] = ode45(@(t,y) dubin_uni(t,y,noise),[0:dt:tfin],[x0; u]);
    # [t2,y2] = ode45(@(t,y) dubin_uni(t,y,0),[0:dt:tfin],[x0; u]);
    # y1 = y1' 
    # y2 = y2'
    # print(x0)
    # print(u)
    # print(np.concatenate((x0,u)).transpose()[0])
    soln1 = solve_ivp(dubin_uni,[0,tfin],np.concatenate((x0,u)).transpose()[0],t_eval=np.linspace(0,tfin,num=(tfin/dt)+1))
    soln1wnoise = soln1.y[0:3].transpose() + np.random.multivariate_normal([0,0,0],Q,int((tfin/dt)+1))
    # print(soln1wnoise[0])
    # self.x = soln1.y[0:2]
    # print(soln1.y)

    R = np.array( ((2*np.pi/180,0), (0,5)) )

    # h = @(x,y) [atan2(y/x);sqrt(x**2 + y**2)] + mvnrnd([0,0],[0.1 0; 0 20])';
    # Gamma = @(t) [1 0 0; 0 1 0; 0 0 1]; % ********** THIS IS probably NOT CORRECT!!!! ********
    Gamma = lambda t: np.eye(3)

    # create EKF filter object
    filt = EKF(dubin_uni,range_bearing,Atilde,Gamma,Htilde,Q,R,dt,x0,P0);


    x_est = [x0]
    x_est_x = [x0[0]]
    x_est_y = [x0[1]]
    x_est_theta = [x0[2]]
    P_est = [P0]
    meas = []

    input_tvec = np.linspace(0,tfin,num=(tfin/dt)+1)
    for i in range(0,len(input_tvec)):
        # propagate filter
        filt.predict(input_tvec[i],u);
        
        # update filter
        # print([soln1.y[0][i],soln1.y[1][i],soln1.y[2][i]])
        y = range_bearing([soln1.y[0][i],soln1.y[1][i],soln1.y[2][i]],R)
        meas.append(y);
        x_curr, P_curr = filt.update(input_tvec[i],y);
        
        # save estimate
        x_est.append(x_curr)
        P_est.append(P_curr)
        x_est_x.append(x_curr[0])
        x_est_y.append(x_curr[1])
        x_est_theta.append(x_curr[2])

    

    import matplotlib.pyplot as plt

    plt.figure(1)
    plt.grid(True)
    plt.plot(soln1.y[0],soln1.y[1])
    # plt.plot(soln1wnoise.y[0],soln1wnoise.y[1])
    plt.plot(x_est_x,x_est_y)
    plt.show()



    # figure
    # hold on; grid on;
    # plot(y1(1,:),y1(2,:))
    # # plot(y2(2,:),y2(2,:))
    # plot(x_est(1,:),x_est(2,:))
    # plot(meas(2,:).*cos(meas(1,:)),meas(2,:).*sin(meas(1,:)),'x')
    # legend('truth','estimate','measurements')
    # xlabel('\xi [m]')
    # ylabel('\eta [m]')
    # title('EKF estimate over circular trajectory')


    # figure

    # subplot(3,1,1)
    # hold on; grid on;
    # plot(input_tvec,x_est(1,:) - y1(1,:))
    # plot(input_tvec,x_est(1,:) - y1(1,:) + 2*sqrt(squeeze(P_est(1,1,:))'),'r--')
    # plot(input_tvec,x_est(1,:) - y1(1,:) - 2*sqrt(squeeze(P_est(1,1,:))'),'r--')
    # plot(input_tvec,zeros(1,length(input_tvec)),'k-.')
    # legend('est','\pm 2\sigma','','truth')
    # xlabel('Time[s]')
    # ylabel('Est error [m]')
    # title('\xi est error')

    # subplot(3,1,2)
    # hold on; grid on;
    # plot(input_tvec,x_est(2,:) - y1(2,:))
    # plot(input_tvec,x_est(2,:) - y1(2,:) + 2*sqrt(squeeze(P_est(2,2,:))'),'r--')
    # plot(input_tvec,x_est(2,:) - y1(2,:) - 2*sqrt(squeeze(P_est(2,2,:))'),'r--')
    # plot(input_tvec,zeros(1,length(input_tvec)),'k-.')
    # legend('est','\pm 2\sigma','','truth')
    # xlabel('Time[s]')
    # ylabel('Est error [m]')
    # title('\eta est error')

    # subplot(3,1,3)
    # hold on; grid on;
    # plot(input_tvec,x_est(3,:) - y1(3,:))
    # plot(input_tvec,x_est(3,:) - y1(3,:) + 2*sqrt(squeeze(P_est(3,3,:))'),'r--')
    # plot(input_tvec,x_est(3,:) - y1(3,:) - 2*sqrt(squeeze(P_est(3,3,:))'),'r--')
    # plot(input_tvec,zeros(1,length(input_tvec)),'k-.')
    # legend('est','\pm 2\sigma','','truth')
    # xlabel('Time[s]')
    # ylabel('Est error [rad]')
    # title('\theta est error')


if __name__ == "__main__":
    test_ekf()