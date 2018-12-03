% Extended Kalman Filter
%
% Ian Loefgren
% 12.1.2018
%
% Implementation of the extended kalman filter
%
% Usage:
%   This implmentation of the extended kalman filter heavily leverages
%   Matlab's ability to create function handles. 
%
%   Call predict() with the time and input to propagate the estimate, then
%   update() with the time and measurement to update the estimate.
%
% Inputs:
%   - f: nonlinear dynamics function handle
%   - h: nonlinear measurement function handle
%   - Atilde: CT dynamics Jacobian function handle
%   - Gamma: noise matrix
%   - Htilde: CT measurement jacobian function handle
%   - dt: timestep
%   - x0: state estimate initialization
%   - P0: state covariance initilization

classdef EKF < handle
    properties
        f
        h
        Atilde
        Gamma
        Htilde
        Q
        R
        dt
        x
        P   
    end
    methods
        function obj = EKF(f,h,Atilde,Gamma,Htilde,Q,R,dt,x0,P0)
            obj.f = f;
            obj.h = h;
            obj.Atilde = Atilde;
            obj.Gamma = Gamma;
            obj.Htilde = Htilde;
            obj.Q = Q;
            obj.R = R;
            obj.dt = dt;
            obj.x = x0;
            obj.P = P0;
        end
        
        function [x_curr,P_curr] = predict(obj,t,u)
            
            % nonlinear fxn eval for state estimate
            [~,soln] = ode45(@(t,y) obj.f(t,y,0),[t,t+obj.dt],[obj.x;u]);
            obj.x = soln(end,1:size(obj.x,1))';
            
            % compute jacobian for current time to compute P approx
            F = eye(size(obj.x,1)) + obj.dt * obj.Atilde(obj.x,u,t);
            Omega = obj.dt * obj.Gamma(t);
            obj.P = F*obj.P*F' + Omega * obj.Q * Omega';
            
            x_curr = obj.x; P_curr = obj.P;
        end
        
        function [x_curr,P_curr] = update(obj,t,meas)
            
            % nonlinear fxn eval for predicted measurement
            y_pred = obj.h(obj.x,zeros(size(meas,1)));
            
            % compute measurement jacobian and approx kalman gain
            H = obj.Htilde(obj.x);
            K = obj.P*H'/(H*obj.P*H' + obj.R);
            
            % update state and covariance estimates
            obj.x = obj.x + K*(meas-y_pred);
            obj.P = (eye(size(obj.P,1)) - K*H)*obj.P;
            
            x_curr = obj.x; P_curr = obj.P;
        end
        
%         function linearize()
%             Hilde
%         end
    end
end