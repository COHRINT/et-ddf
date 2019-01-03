#!/usr/bin/env python

__author__ = "Ian Loefgren"
__date__ = "12.28.2018"

"""
Python implementation of the Unscented Kalman Filter.
"""

import numpy as np


class UKF(object):

    def __init__(self,f,h,Q,R,dt,x0,P0,kappa,alpha,beta):
        self.f = f
        self.h = h
        self.R = R
        self.dt = dt
        self.n = x0.size[0]
        self.x = x0
        self.P = P0
        self.kappa = kappa
        self.alpha = alpha
        self.beta = beta
        self.lamb = (self.alpha^2 *(self.n+self..kappa)-self.n)
        self.wm0 = self.lamb/(self.n+self.lamb);
        self.wc0 = self.lamb/(self.n+self.lamb) + 1 - self.alpha^2 + self.beta;
        self.wm = 1/(2*(self.n+self.lamb));
        self.wc = self.wm;

    def predict(self,t,u):

        S = np.linalg.cholesky(self.P)

        sigma_points = zeros(self.n,2*self.n+1);
        prop_sigma_points = zeros(self.n,2*self.n+1);
        x_sum = zeros(self.n,1);
        P_sum = zeros(self.n);
        
        # propagate chi_0, point at mean
        sigma_points(:,1) = self.x;
        [~,soln] = ode45(@(t,y) self.f(t,y,u,[0 0 0]),[t,t+self.dt],sigma_points(:,1));
        prop_sigma_points(:,1) = soln(end,1:size(self.x,1))';
        x_sum = x_sum + self.wm0 * prop_sigma_points(:,1);
#             x_sum = x_sum + 10*(1/(2*self.n+1)) * prop_sigma_points(:,1);
        
        # propagate other points
        for i=1:self.n
            sigma_points(:,i+1) = self.x + sqrt(self.n+self.lamb)*S(i,:)';
            sigma_points(:,self.n+i+1) = self.x - sqrt(self.n+self.lamb)*S(i,:)';
            
            [~,soln] = ode45(@(t,y) self.f(t,y,u,[0 0 0]),[t,t+self.dt],sigma_points(:,i+1),self.options);
            prop_sigma_points(:,i+1) = soln(end,1:size(self.x,1))';
            x_sum = x_sum + self.wm * prop_sigma_points(:,i+1);
#                 x_sum = x_sum + ((2*self.n-9)/((2*self.n+1)*2*self.n)) * prop_sigma_points(:,i+1);
            
            [~,soln] = ode45(@(t,y) self.f(t,y,u,[0 0 0]),[t,t+self.dt],sigma_points(:,self.n+i+1),self.options);
            prop_sigma_points(:,self.n+i+1) = soln(end,1:size(self.x,1))';
            x_sum = x_sum + self.wm * prop_sigma_points(:,self.n+i+1);
#                 x_sum = x_sum + ((2*self.n-9)/((2*self.n+1)*2*self.n)) * prop_sigma_points(:,i+1);
        
        P_sum = P_sum + self.wc0 * (prop_sigma_points(:,1)-x_sum)*(prop_sigma_points(:,1)-x_sum)';
#             P_sum = P_sum + 10*(1/(2*self.n+1)) * (prop_sigma_points(:,1)-x_sum)*(prop_sigma_points(:,1)-x_sum)';
        
        for i=1:2*self.n
            P_sum = P_sum + self.wc * (prop_sigma_points(:,i+1)-x_sum)*(prop_sigma_points(:,i+1)-x_sum)';
#                 P_sum = P_sum + ((2*self.n-9)/((2*self.n+1)*2*self.n)) * (prop_sigma_points(:,i+1)-x_sum)*(prop_sigma_points(:,i+1)-x_sum)';
        end
        
        P_sum = P_sum + self.Q;
        
        
        xcurr = x_sum;
        self.x = x_sum;
        
        self.P = 0.5*P_sum' + 0.5*P_sum;
        Pcurr = self.P;
