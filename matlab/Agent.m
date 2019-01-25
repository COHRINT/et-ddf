%%% General "Agent" class for event-triggered cooperative localization

classdef Agent < handle
    properties
        agent_id
        connections
        local_filter
        common_estimates
        true_state
        ci_trigger_cnt
        msgs_sent
        total_msgs
        msg_success_prob
        
%         sensors
    end
    
    methods
        function obj = Agent(agent_id,connections,local_filter,common_estimates,x_true,msg_drop_prob)
            
            % agent id
            obj.agent_id = agent_id;
            
            % ids of connections
            obj.connections = connections;
            
            % filter object for local estimation
            obj.local_filter = local_filter;
            
            % struct with ids of connections and filters estimating common
            obj.common_estimates = common_estimates;
            
            % true starting position
            obj.true_state = x_true;
            
            % CI trigger count
            obj.ci_trigger_cnt = 0;
            
            obj.msgs_sent = 0;
            obj.total_msgs = 0;
            
            obj.msg_success_prob = 1-msg_drop_prob;
            
            % struct of sensors available to agent
%             obj.sensors = sensors;
        end
        
%         function update(obj,input,local_measurements,received_measurements)
%             
%             % update local filter, and determine if local measurements
%             % should be sent to connections
%             outgoing = obj.process_local_measurements(input,local_measurements);
%             
%             % send measurements to connections
%             obj.send_measurements(outgoing);
%             
%             % update common estimates with received measurements from
%             % connections
%             obj.process_recevied_measurements(received_measurements);
%             
%         end
        
        function [outgoing_msgs] = process_local_measurements(obj,input,local_measurements)
            
%             agent_loc = find(obj.connections>obj.agent_id);
            agent_loc = find(sort([obj.connections,obj.agent_id]) == obj.agent_id);
            
            % predict and update local filter
            input_vec = zeros(size(obj.local_filter.G,2),1);
            input_vec(2*(agent_loc-1)+1:2*(agent_loc-1)+2,1) = input;
%             input_vec(1:length(input)) = input;
            obj.local_filter.predict(input_vec);
            if (sum(isnan(obj.local_filter.x)) > 0) || (sum(isinf(obj.local_filter.x)) > 0)
                disp('help!')
            end
            
            src_loc_list = zeros(length(local_measurements),1);
            dest_loc_list = zeros(length(local_measurements),1);
            
            for i=1:length(local_measurements)
                if isempty(local_measurements{i}.type)
                    disp('breakpnt')
                end
%                 fprintf('AGENT LEVEL: Msg type, %s \t Msg src id, %i \t Msg dest id, %i \t Obj agent id, %i\n',local_measurements{i}.type,local_measurements{i}.src,local_measurements{i}.dest,obj.agent_id);

                % determine location of src and destination elements of
                % state vector
%                 src_loc = find(obj.connections == local_measurements{i}.src);
                src_loc = find(sort([obj.connections,obj.agent_id]) == local_measurements{i}.src);
                if isempty(src_loc)
                    src_loc = agent_loc;
                end
                src_loc_list(i) = src_loc;
%                 dest_loc = find(obj.connections == local_measurements{i}.dest)+1;
                dest_loc = find(sort([obj.connections,obj.agent_id]) == local_measurements{i}.dest);
                if isempty(dest_loc)
                    dest_loc = agent_loc;
                end
                dest_loc_list(i) = dest_loc;

                if strcmp(local_measurements{i}.type,'abs')
%                     obj.local_filter.explicit_update({local_measurements{i}.data},local_measurements{i}.type,local_measurements{i}.src,local_measurements{i}.dest);
                    obj.local_filter.explicit_update({local_measurements{i}.data},local_measurements{i}.type,src_loc,dest_loc);
                elseif strcmp(local_measurements{i}.type,'rel')
%                     obj.local_filter.explicit_update({local_measurements{i}.data},local_measurements{i}.type,local_measurements{i}.src,local_measurements{i}.dest);
                    obj.local_filter.explicit_update({local_measurements{i}.data},local_measurements{i}.type,src_loc,dest_loc);
                end
                if (sum(isnan(obj.local_filter.x)) > 0) || (sum(isinf(obj.local_filter.x)) > 0)
                    disp('help!')
                end
            end
            
            outgoing_msgs = {};
            
            % loop through common estimate filters and predict and update
            
%             outgoing = zeros(length(obj.common_estimates),size(local_measurements,1));
            
            for i=1:length(obj.common_estimates)
                
                % propagate common estimate
                obj.common_estimates{i}.predict(zeros(size(obj.common_estimates{i}.G,2),1));
                
                % for each connection, threshold local measurements
                for j=1:length(obj.common_estimates{i}.connection)
                    
                    [transmit,type,msg] = obj.common_estimates{i}.threshold(local_measurements,src_loc_list,dest_loc_list); 
                    
%                     disp(transmit)
                    
                    
                    for k=1:size(msg,2)
                        obj.common_estimates{i}.total_msg = obj.common_estimates{i}.total_msg+1;
                        if all(transmit(:,k)) % explicit update
                            ii = i;
%                             for ii=1:length(obj.common_estimates)
%                                 if obj.common_estimates{ii}.connection == src
%                                     disp(type(1,j))
                                if dest_loc_list(k) > agent_loc
                                    src_id = 1;
                                    dest_id = 2;
                                else
                                    src_id = 2;
                                    dest_id = 1;
                                end
                                obj.common_estimates{ii}.explicit_update(msg(k),type(1,k),src_id,dest_id);
%                                 obj.common_estimates{ii}
%                                 end
%                                 obj.msgs_sent = obj.msgs_sent + 1;
%                                 obj.total_msgs = obj.total_msgs + 1;
                                obj.common_estimates{ii}.msg_sent = obj.common_estimates{ii}.msg_sent+1;
%                                 obj.common_estimates{ii}.total_msg = obj.common_estimates{ii}.total_msg+1;
%                             end
%                         end
                        else % implicit update
                            ii = i;
                            
                            if dest_loc_list(k) > agent_loc
                                x_local = [obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4); obj.local_filter.x(4*(dest_loc_list(k)-1)+1:4*(dest_loc_list(k)-1)+4)];
                                P_local = obj.local_filter.P([4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(dest_loc_list(k)-1)+1:4*(dest_loc_list(k)-1)+4],...
                                                [4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(dest_loc_list(k)-1)+1:4*(dest_loc_list(k)-1)+4]);
                                src_id = 1;
                                dest_id = 2;
                            else
                                x_local = [obj.local_filter.x(4*(dest_loc_list(k)-1)+1:4*(dest_loc_list(k)-1)+4); obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4)];
                                P_local = obj.local_filter.P([4*(dest_loc_list(k)-1)+1:4*(dest_loc_list(k)-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4],...
                                                [4*(dest_loc_list(k)-1)+1:4*(dest_loc_list(k)-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4]);
                                src_id = 2;
                                dest_id = 1;
                            end
%                             for ii=1:length(obj.common_estimates)
%                                 if obj.common_estimates{ii}.connection == src
%                                 obj.common_estimates{ii}.implicit_update(msg(k),type(k),obj.agent_id,obj.local_filter.x,obj.local_filter.P);
%                                 if dest_loc_list(i) > agent_loc
%                                     x_local = [obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4); obj.local_filter.x(4*(dest_loc_list(i)-1)+1:4*(dest_loc_list(i)-1)+4)];
%                                     P_local = obj.local_filter.P([4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(dest_loc_list(i)-1)+1:4*(dest_loc_list(i)-1)+4],...
%                                                                     [4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(dest_loc_list(i)-1)+1:4*(dest_loc_list(i)-1)+4]);
%                                 else
%                                     x_local = [obj.local_filter.x(4*(dest_loc_list(i)-1)+1:4*(dest_loc_list(i)-1)+4); obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4)];
%                                     P_local = obj.local_filter.P([4*(dest_loc_list(i)-1)+1:4*(dest_loc_list(i)-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4],...
%                                                                     [4*(dest_loc_list(i)-1)+1:4*(dest_loc_list(i)-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4]);
%                                 end
                                    
                                obj.common_estimates{ii}.implicit_update(msg(k),type(k),src_id,dest_id,x_local,P_local);
%                                 end
%                                 obj.total_msgs = obj.total_msgs + 1;
                                
%                             end
                        end
                    end
                    
                    % create outgoing msg struct
                    outgoing_msgs{end+1} = struct('src',obj.agent_id,'dest',obj.common_estimates{i}.connection,...
                        'status',{transmit},'type',{type},'data',{msg});
                end
            end
            if (sum(isnan(obj.local_filter.x)) > 0) || (sum(isinf(obj.local_filter.x)) > 0)
                disp('help!')
            end
        end
        
        function process_received_measurements(obj,inbox)
            
%             agent_loc = find(obj.connections>obj.agent_id);
            agent_loc = find(sort([obj.connections,obj.agent_id]) == obj.agent_id);
            
            % based on inbox measurements perform implicit and explicit
            % updates
            for i=1:length(inbox)
                if ~isempty(inbox{i})
                    src = inbox{i}.src;
                    dest = inbox{i}.dest;
                    status = inbox{i}.status;
                    type = inbox{i}.type;
                    data = inbox{i}.data;
                    
%                     src_loc = find(obj.connections == src);
                    src_loc = find(sort([obj.connections,obj.agent_id]) == src);
                    if isempty(src_loc)
                        src_loc = agent_loc;
                    end
%                     src_loc_list(i) = src_loc;
%                     dest_loc = find(obj.connections == dest);
                    dest_loc = find(sort([obj.connections,obj.agent_id]) == dest);
                    if isempty(dest_loc)
                        dest_loc = agent_loc;
                    end
%                     dest_loc_list(i) = dest_loc;
                    
                    
                    for j=1:size(data,2)
                        if (all(status(:,j))) && (binornd(1,obj.msg_success_prob)) % explicit update
                            
                            if (sum(isnan(obj.local_filter.x)) > 0) || (sum(isinf(obj.local_filter.x)) > 0)
                                disp('help!')
                            end
                            
                            obj.local_filter.explicit_update(data(j),type(1,j),src_loc,dest_loc);
                            
                            if (sum(isnan(obj.local_filter.x)) > 0) || (sum(isinf(obj.local_filter.x)) > 0)
                                disp('help!')
                            end
                            for k=1:length(obj.common_estimates)
                                if obj.common_estimates{k}.connection == src
%                                     disp(type(1,j))
                                    if src_loc > agent_loc % agent_loc is the the dest_loc
                                        src_loc_new = 2;
                                        dest_loc_new = 1;
                                    else
                                        src_loc_new = 1;
                                        dest_loc_new = 2;
                                    end
                                    obj.common_estimates{k}.explicit_update(data(j),type(1,j),src_loc_new,dest_loc_new);
                                end
                            end
%                         end
                        else % implicit update
                            local_est = obj.local_filter.x; local_cov = obj.local_filter.P;
                            
%                             if dest_loc > agent_loc
%                                     x_local = [obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4); obj.local_filter.x(4*(dest_loc-1)+1:4*(dest_loc-1)+4)];
%                                     P_local = obj.local_filter.P([4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(dest_loc-1)+1:4*(dest_loc-1)+4],...
%                                                                     [4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(dest_loc-1)+1:4*(dest_loc-1)+4]);
%                             else
%                                 x_local = [obj.local_filter.x(4*(dest_loc-1)+1:4*(dest_loc-1)+4); obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4)];
%                                 P_local = obj.local_filter.P([4*(dest_loc-1)+1:4*(dest_loc-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4],...
%                                                             [4*(dest_loc-1)+1:4*(dest_loc-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4]);
%                             end
                            
                            if (sum(isnan(local_est)) > 0) || (sum(isinf(local_est)) > 0)
                                disp('help!')
                            end
                            obj.local_filter.implicit_update(data(j),type(j),src_loc,dest_loc,local_est,local_cov);
                            if (sum(isnan(obj.local_filter.x)) > 0) || (sum(isinf(obj.local_filter.x)) > 0)
                                disp('help!')
                            end
                            for k=1:length(obj.common_estimates)
                                if obj.common_estimates{k}.connection == src
                                    if src_loc > agent_loc
                                        x_local = [obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4); obj.local_filter.x(4*(src_loc-1)+1:4*(src_loc-1)+4)];
                                        P_local = obj.local_filter.P([4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(src_loc-1)+1:4*(src_loc-1)+4],...
                                                        [4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(src_loc-1)+1:4*(src_loc-1)+4]);
                                        src_loc = 2;
                                        dest_loc = 1;
                                        
                                    else
                                        x_local = [obj.local_filter.x(4*(src_loc-1)+1:4*(src_loc-1)+4); obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4)];
                                        P_local = obj.local_filter.P([4*(src_loc-1)+1:4*(src_loc-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4],...
                                                            [4*(src_loc-1)+1:4*(src_loc-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4]);
                                        src_loc = 1;
                                        dest_loc = 2;
                                    end
                                    obj.common_estimates{k}.implicit_update(data(j),type(j),src_loc,dest_loc,x_local,P_local);
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end