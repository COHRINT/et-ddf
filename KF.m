%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kalman Filter implementation
%
% Ian Loefgren
% Last modified: 11.11.2018
%
% Implements linear kalman filter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef KF < handle
    properties
        F
        G
        H
        M
        Q
        R
        x
        P
    end
    methods
        function obj = KF(F_,G_,H_,M_,Q_,R_,x0,P0)
            obj.F = F_;
            obj.G = G_;
            obj.H = H_;
            obj.M = M_;
            obj.Q = Q_;
            obj.R = R_;
            obj.x = x0;
            obj.P = P0;
        end
        
        function [x_curr,P_curr] = predict(obj,u)
            
            x_curr = obj.F*obj.x + obj.G*u;
            P_curr = obj.F*obj.P*obj.F' + obj.Q;
            
            obj.x = x_curr;
            obj.P = P_curr;
        end
        
        function [x_curr,P_curr] = update(obj,meas)
            for i=1:size(meas,2)
                
                % extract actual measurement
                meas_val = meas(:,i);
                
                % compute predicted measurement and innovation
                meas_pred = obj.H*obj.x;
                innov = meas_val - meas_pred;
                
                % compute Kalman gain
                K = obj.P*obj.H'/(obj.H*obj.P*obj.H' + obj.R);
                
                % update state
                x_curr = obj.x + K*innov;
                % update covariance
                P_curr = (eye(size(obj.F,1))-K*obj.H)*obj.P;
                
                obj.x = x_curr;
                obj.P = P_curr;
            end
        end
                   
    end
end