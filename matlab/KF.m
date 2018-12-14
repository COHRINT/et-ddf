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
        R_rel
        R_abs
        x
        P
        state_history
        cov_history
        agent_id
    end
    methods
        function obj = KF(F_,G_,H_,M_,Q_,R_abs,R_rel,x0,P0,agent_id)
            obj.F = F_;
            obj.G = G_;
            obj.H = H_;
            obj.M = M_;
            obj.Q = Q_;
            obj.R_rel = R_rel;
            obj.R_abs = R_abs;
            obj.x = x0;
            obj.P = P0;
            obj.state_history(:,1) = x0;
            obj.cov_history(:,:,1) = P0;
            obj.agent_id = agent_id;
        end
        
        function [x_curr,P_curr] = predict(obj,u)
            
            x_curr = obj.F*obj.x + obj.G*u;
            P_curr = obj.F*obj.P*obj.F' + obj.Q;
            
            obj.x = x_curr;
            obj.P = P_curr;
            obj.state_history(:,end+1) = x_curr;
            obj.cov_history(:,:,end+1) = P_curr;
        end
        
        function [x_curr,P_curr] = update(obj,meas,meas_type,src_id,rel_agent_id)
            for i=1:size(meas,2)
%                 fprintf('FILTER LEVEL: Msg type, %s \t Msg src id, %i \t Msg rel angent id, %i \t Obj agent id, %i\n',meas_type,src_id,rel_agent_id,obj.agent_id);
                H = zeros(2,size(obj.F,1));
                if meas_type == 'abs'
                    H(1,4*(src_id-1)+1) = 1; H(2,4*(src_id-1)+3) = 1;
                    R = obj.R_abs;
                elseif meas_type == 'rel'
                    H(1,4*(src_id-1)+1) = 1; H(2,4*(src_id-1)+3) = 1;
                    H(1,4*(rel_agent_id-1)+1) = -1; H(2,4*(rel_agent_id-1)+3) = -1;
                    R = obj.R_rel;
                end
                
                % extract actual measurement
                meas_val = meas(:,i);
                
                % compute predicted measurement and innovation
                meas_pred = H*obj.x;
                innov = meas_val - meas_pred;
                
                % compute Kalman gain
                K = obj.P*H'/(H*obj.P*H' + R);
                
                % update state
                x_curr = obj.x + K*innov;
                % update covariance
                P_curr = (eye(size(obj.F,1))-K*H)*obj.P;
                
                
                if (sum(isnan(x_curr)) ~= 0) || (sum(isinf(x_curr)))
                    disp('NaN!!!!')
                end
                
                obj.x = x_curr;
                obj.P = P_curr;
                obj.state_history(:,end) = x_curr;
                obj.cov_history(:,:,end) = P_curr;
            end
        end
                   
    end
end