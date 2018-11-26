%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Event-triggered Kalman Filter implementation
%
% Ian Loefgren
% Last modified: 11.11.2018
%
% Implements an event-triggered linear kalman filter with implicit
% measurement fusion.
%
% Usage:
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef ETKF < handle
    properties
        F
        G
        H
        M
        Q
        R
        x
        P
        xpred
        delta = 1;
        msg_sent = 0;
    end
    methods
        function obj = ETKF(F_,G_,H_,M_,Q_,R_,x0,P0,delta)
            obj.F = F_;
            obj.G = G_;
            obj.H = H_;
            obj.M = M_;
            obj.Q = Q_;
            obj.R = R_;
            obj.x = x0;
            obj.P = P0;
            obj.delta = delta;
        end
        
        function [x_curr,P_curr] = predict(obj,u)
            
            x_curr = obj.F*obj.x + obj.G*u;
            P_curr = obj.F*obj.P*obj.F' + obj.Q;
            
            obj.x = x_curr;
            obj.xpred = x_curr;
            obj.P = P_curr;
        end
        
        function [x_curr,P_curr] = update(obj,meas)
            for i=1:size(meas,2)
                
                % extract actual measurement
                meas_val = meas(:,i);
                
                % loop through elements of measurement
                for j=1:size(meas_val,1)
                    % compute predicted measurement and innovation
                    meas_pred = obj.H(j,:)*obj.x;
                    innov = meas_val(j) - meas_pred;
                    
                    % if surprising enough, perform explicit update
                    if abs(innov) > obj.delta
                        [x_curr,P_curr] = obj.explicit_update(meas_val(j),j);
                    elseif abs(innov) <= obj.delta
                        [x_curr,P_curr] = obj.implicit_update(meas_val(j),j);
                    end
                end
   
            end
        end
        
        function [x_curr,P_curr] = explicit_update(obj,meas_val,i)
            
            % compute predicted measurement and innovation
            meas_pred = obj.H(i,:)*obj.x;
            innov = meas_val - meas_pred;
            
            % compute Kalman gain
            K = obj.P*obj.H(i,:)'/(obj.H(i,:)*obj.P*obj.H(i,:)' + obj.R(i,i));

            % update state
            x_curr = obj.x + K*innov;
            % update covariance
            P_curr = (eye(size(obj.F,1))-K*obj.H(i,:))*obj.P;

            % update filter values
            obj.x = x_curr;
            obj.P = P_curr;
            obj.msg_sent = obj.msg_sent + 1;
        end
        
        function [x_curr,P_curr] = implicit_update(obj,meas_val,i)
            
            % implicit measurement pdf fxn handle
            phi = @(z) (1/sqrt(2*pi)) * exp(-0.5*z^2);
            Qfxn = @(z) normcdf(z,0,1,'upper');

            mu = obj.H(i,:)*obj.x - obj.H(i,:)*obj.xpred;
            Qe = obj.H(i,:)*obj.P*obj.H(i,:)' + obj.R(i,i);
            % a = h(x_ref) - h(xbar(k))
            a = meas_val-obj.H(i,:)*obj.x;

            arg1 = -obj.delta+a-mu/sqrt(Qe);
            arg2 = obj.delta+a-mu/sqrt(Qe);

            zbar = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2)))*sqrt(Qe);
            dcal = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2))^2) - ...
                        ((arg1)*phi(arg1)-arg2*phi(arg2)/(Qfxn(arg1)-Qfxn(arg2)));

            K = obj.P * obj.H(i,:)'*inv(obj.H(i,:)*obj.P*obj.H(i,:)' + obj.R(i,i));
%             Kimplicit(:,i) = Kevent;
            x_curr = obj.x + K*zbar;
            invquant = inv(obj.H(i,:)*obj.P*obj.H(i,:)' + obj.R(i,i));
            P_curr = obj.P - dcal*obj.P*obj.H(i,:)'*invquant*obj.H(i,:)*obj.P;
%             P(:,:,i) = P(:,:,i)-dcal*K*P(:,:,i);
%             P(:,:,i) = (eye(2)-dcal*Kevent*Hevent)*Hevent*P(:,:,i);

            % update filter values
            obj.x = x_curr;
            obj.P = P_curr;    
        end
    end
end