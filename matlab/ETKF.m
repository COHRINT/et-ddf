%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Event-triggered Kalman Filter implementation
%
% Ian Loefgren
% Last modified: 3.21.2019
%
% Implements an event-triggered linear kalman filter with implicit
% measurement fusion.
%
% Usage:
%   - Filter is updated using msg_update() and passing a measurement-type
%   message, and mean and cov of current est
%   - Based on status field of message, elements are updated explicitly or
%   implicitly
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef ETKF < handle
    properties
        F
        G
        H
        M
        Q
        R_abs
        R_rel
        x
        P
        xpred
        delta = 1
        msg_sent
        total_msg
        agent_id
        connection
        meas_connection
        state_history
        cov_history
        innovation_history = [];
    end
    methods
        function obj = ETKF(F_,G_,H_,M_,Q_,R_abs,R_rel,x0,P0,delta,agent_id,connections,meas_connections)
            obj.F = F_;
            obj.G = G_;
            obj.H = H_;
            obj.M = M_;
            obj.Q = Q_;
            obj.R_abs = R_abs;
            obj.R_rel = R_rel;
            obj.x = x0;
            obj.P = P0;
            obj.delta = delta;
            obj.msg_sent = 0;
            obj.total_msg = 0;
            obj.agent_id = agent_id;
            obj.connection = connections;
            obj.meas_connection = meas_connections;
            obj.state_history(:,1) = x0;
            obj.cov_history(:,:,1) = P0;
        end
        
        function [loc] = get_location(obj,id)
            % get location from agent id
            
            loc = find(sort([obj.agent_id,obj.connection]) == id);
            
        end
        
        function [id] = get_id(obj,loc)
            % get id from location in state estimate
            
            ordered_ids = sort([obj.agent_id,obj.connection]);
            id = ordered_ids(loc);
            
        end
        
        function [x_curr,P_curr] = predict(obj,u)
            
            x_curr = obj.F*obj.x + obj.G*u;
            P_curr = obj.F*obj.P*obj.F' + obj.Q;
            
            obj.x = x_curr;
            obj.xpred = x_curr;
            obj.P = P_curr;

        end
        
        function [src,dest,target,outgoing_status,type,outgoing_data] = threshold(obj,msg)
            % thresholds each measurement component against pred and
            % creates boolean vector of whether to trasmit each component
            % or not
            
            % unpack message
            src = msg.src;
            dest = msg.dest;
            target = msg.target;
            status = msg.status;
            type = msg.type;
            data = msg.data;
            
            % get locations in state estimate of measurement src and target
            src_loc = obj.get_location(src);
            target_loc = obj.get_location(target);
            
            % initialize new status and data arrays
            outgoing_status = zeros(1,length(data));
            outgoing_data = [];
            
            % threshold each measurement element induvidually
            for i=1:length(data)
                
                % create appropriate measurement function
                H = zeros(1,size(obj.F,1));
                if strcmp(type,"abs")
                    H(1,4*(src_loc-1)+(i-1)*2+1) = 1; %H(2,4*(src_loc-1)+3) = 1;
                elseif strcmp(type,"rel")
                    H(1,4*(src_loc-1)+(i-1)*2+1) = 1; %H(2,4*(src_loc-1)+3) = 1;
                    H(1,4*(target_loc-1)+(i-1)*2+1) = -1; %H(2,4*(dest_loc-1)+3) = -1;
                end
                
                % filter predicted measurement
                meas_pred = H*obj.x;
                % innovation
                innov = data(i) - meas_pred;
                
                if abs(innov) > obj.delta
                    outgoing_status(i) = 1;
                    outgoing_data = [outgoing_data,data(i)];
                end  
            end    
        end
        
        function [x_curr,P_curr] = explicit_update(obj,src_id,target_id,type,meas_val,data_idx)
            
            i=data_idx;
            H = zeros(1,size(obj.F,1));
            if strcmp(type,"abs")
                H(1,4*(src_id-1)+(i-1)*2+1) = 1; %H(2,4*(src_id-1)+3) = 1;
                R = obj.R_abs(1,1);
            elseif strcmp(type,"rel")
                H(1,4*(src_id-1)+(i-1)*2+1) = 1; %H(2,4*(src_id-1)+3) = 1;
                H(1,4*(target_id-1)+(i-1)*2+1) = -1; %H(2,4*(dest_id-1)+3) = -1;
                R = obj.R_rel(1,1);
            end
            
            % compute predicted measurement and innovation
            meas_pred = H*obj.x;
            innov = meas_val - meas_pred;
            obj.innovation_history(i,end+1) = innov;
            
            % compute Kalman gain
            K = obj.P*H'/(H*obj.P*H' + R);

            % update state
            x_curr = obj.x + K*innov;
            
            % update covariance
            P_curr = (eye(size(obj.F,1))-K*H)*obj.P*(eye(size(obj.F,1))-K*H)' + K*R*K';
            
            % update filter values
            obj.x = x_curr;
            obj.P = 0.5*P_curr + 0.5*P_curr';

        end
        
        function [x_curr,P_curr] = implicit_update(obj,src_id,target_id,type,x_local,P_local,data_idx)
            
%             src_id = 1;
            
%             R_imp = 10;
            if isempty(type{1})
                type{1} = 'abs';
            end
            
            if isempty(src_id) || isempty(target_id)
                breakpoint = 0;
            end
            
            % implicit measurement pdf fxn handle
            phi = @(z) (1/sqrt(2*pi)) * exp(-0.5*z^2);
            Qfxn = @(z) normcdf(z,0,1,'upper');
            
%             for i=1:length(meas_val{1})
            i=data_idx;
                
                H = zeros(1,size(obj.F,1));
                H_local = zeros(1,size(x_local,1));
                if strcmp(type,"abs")
                    H(1,4*(src_id-1)+(i-1)*2+1) = 1; %H(2,4*(src_id-1)+2*i) = 1;
                    H_local(1,4*(src_id-1)+(i-1)*2+1) = 1;
                    R = obj.R_rel(i,i);
                elseif strcmp(type,"rel")
                    H(1,4*(src_id-1)+(i-1)*2+1) = 1; %H(2,4*(src_id-1)+3) = 1;
                    H(1,4*(target_id-1)+(i-1)*2+1) = -1; %H(2,4*(obj.agent_id-1)+3) = -1;
%                     H_local(1,(i-1)*2+1) = 1;
%                     H_local(1,(i-1)*2+3) = -1;
%                     H_local(1,1) = 1;
%                     H_local(1,3) = -1;
                    H_local = H;
                    R = obj.R_rel(i,i);
                end

%                 mu = H*obj.x - H_local*x_local;
                mu = H*obj.x - H_local*obj.xpred;
                
                Qe = H_local*P_local*H_local' + R;
                
%                 a = H*obj.x - H_local*x_local;
                a = H*x_local - H_local*obj.xpred;

                arg1 = (-obj.delta+a-mu)/sqrt(Qe);
                arg2 = (obj.delta+a-mu)/sqrt(Qe);

                zbar = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2)))*sqrt(Qe);
                dcal = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2))^2) - ...
                            ((arg1)*phi(arg1)-arg2*phi(arg2)/(Qfxn(arg1)-Qfxn(arg2)));

                K = obj.P * H'/(H*obj.P*H' + R);
                x_curr = obj.x + K*zbar;
                invquant = H*obj.P*H' + R;
                P_curr = obj.P - dcal*obj.P*H'/invquant*H*obj.P;

                % update filter values
                obj.x = x_curr;
                obj.P = 0.5*P_curr + 0.5*P_curr';

%                 obj.state_history(:,end) = obj.x;
%                 obj.cov_history(:,:,end) = obj.P;
%             end
        end
        
        function msg_update(obj,msg,x_local,P_local)
            % general update function to deal with measurements
            
            % check if x and P_local are provided, if not, local update,
            % will be all explicit, create placeholders so matlab doesn't complain
            if nargin < 3
                x_local = [];
                P_local = [];
            end
            
            % unpack message
            src = msg.src;
            dest = msg.dest;
            target = msg.target;
            status = msg.status;
            type = msg.type;
            data = msg.data;
            
            % get locations in state estimate of measurement src and dest
            src_loc = obj.get_location(src);
            target_loc = obj.get_location(target);
            
            if (isempty(src_loc) || isempty(target_loc))
                breakpoint = 0;
            end
            
            % fuse all measurement elements appropriately
            data_cnt = 1;
            for i=1:length(status)
                
                % if status for element is true, explicit update
                if status(i)
                    obj.explicit_update(src_loc,target_loc,type,data(data_cnt),i);
                    data_cnt = data_cnt + 1;
                    
                % otherwise, implicit update
                else
                    obj.implicit_update(src_loc,target_loc,type,x_local,P_local,i);
                end  
            end    
        end    
    end
end