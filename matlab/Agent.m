%%% General "Agent" class for event-triggered cooperative localization

classdef Agent < handle
    properties
        agent_id
        connections
        num_states
        local_filter
        common_estimates
        true_state
        ci_trigger_cnt
        msgs_sent
        total_msgs
        msg_success_prob
        tau_goal
        tau
        connection_taus
        epsilon_1
        epsilon_2
        tau_history = []
        
%         sensors
    end
    
    methods
        function obj = Agent(agent_id,connections,local_filter,common_estimates,x_true,msg_drop_prob,tau_goal,tau)
            
            % agent id
            obj.agent_id = agent_id;
            
            % ids of connections
            obj.connections = connections;
            
            % number of states per platform
            obj.num_states = 4;
            
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
            
            % CI thresholding data
            obj.tau_goal = tau_goal;
            obj.tau = tau;
            obj.connection_taus = tau*ones(length(obj.connections),1);
            obj.epsilon_1 = 0.1;
            obj.epsilon_2 = 0.01;
            
            % struct of sensors available to agent
%             obj.sensors = sensors;
        end
        
        function [loc,idx] = get_location(obj,id)
            % get location of agent in state estimate given id, and acutal
            % estimate indicies
            
            loc = find(sort([obj.agent_id,obj.connections]) == id);
            idx = obj.num_states*(loc-1)+1:obj.num_states*(loc-1)+obj.num_states;
            
        end
        
        function [id] = get_id(obj,loc)
            % get id of agent from location in state estimate
            
            ordered_ids = sort([obj.agent_id,obj.connections]);
            id = ordered_ids(loc);
            
        end
        
        function [outgoing] = process_local_measurements(obj,input,local_measurements) 
            
            % get agent location in state estimate
            [agent_loc,agent_idx] = obj.get_location(obj.agent_id);
            
            % turn input into correctly sized vector for prediction
            input_vec = zeros(size(obj.local_filter.G,2),1);
            input_vec(2*(agent_loc-1)+1:2*(agent_loc-1)+2,1) = input;
            
            % predict and update local filter, and common estimates
            obj.local_filter.predict(input_vec);
            for i=1:length(obj.common_estimates)
                % propagate common estimate, giving no input
                obj.common_estimates{i}.predict(zeros(size(obj.common_estimates{i}.G,2),1));
            end
            
            % create outgoing msg queue
            outgoing = {};
            
            % process each collected measurement
            for i=1:length(local_measurements)
                
                msg = local_measurements{i};

                % source of local measurements is agent
                assert(msg.src == obj.agent_id);

                % update local filter
                obj.local_filter.msg_update(msg);
                
                obj.total_msgs = obj.total_msgs + 2;
                
                % threshold measurement and update common estimate
                for j=1:length(obj.common_estimates)
                    
                    if (local_measurements{i}.dest == msg.src) && (strcmp(msg.type,"abs"))
                        dest = obj.common_estimates{j}.connection;
                        msg = gen_msg(msg.src,dest,msg.status,msg.type,msg.data);                        
                    end
                    
                    [~,src_idx] = obj.get_location(msg.src);
                    [~,dest_idx] = obj.get_location(msg.dest);

                    
                    % find common estimate associated with msg destination
                    if obj.common_estimates{j}.connection == msg.dest
                        
                        % threshold measurement
                        [src_,dest_,status_,type_,data_] = obj.common_estimates{j}.threshold(msg);
                        % generate msg from threshold results
                        msg = gen_msg(src_,dest_,status_,type_,data_);
                        
                        % add measurement elements to be sent to sent cnt
                        obj.msgs_sent = obj.msgs_sent + sum(status_);
                        
                        % update common estimate with thresholding result
                        x_local = obj.local_filter.x(sort([src_idx,dest_idx]));
                        P_local = obj.local_filter.P(sort([src_idx,dest_idx]),sort([src_idx,dest_idx]));
                        obj.common_estimates{j}.msg_update(msg,x_local,P_local);
                        
                        % add msg to outgoing msg queue
                        outgoing{end+1} = msg;
                        
                    end
                end              
            end
        end
            
%             outgoing_msgs = {};
            
            % loop through common estimate filters and predict and update
            
%             outgoing = zeros(length(obj.common_estimates),size(local_measurements,1));
            
%             for i=1:length(obj.common_estimates)
%                 
%                 % propagate common estimate, giving no input
%                 obj.common_estimates{i}.predict(zeros(size(obj.common_estimates{i}.G,2),1));
%                 
%                 [transmit,type,msg] = obj.common_estimates{i}.threshold(local_measurements,src_loc_list,dest_loc_list); 
%                 
%                 % for each connection, threshold local measurements
%                 for j=1:length(obj.common_estimates{i}.connection)
%                     
%                     for k=1:size(msg,2)
%                         
%                         obj.common_estimates{i}.total_msg = obj.common_estimates{i}.total_msg+1;
%                         
%                         if all(transmit(:,k)) % explicit update
%                             ii = i;
% 
%                                 if dest_loc_list(k) > agent_loc
%                                     src_id = 1;
%                                     dest_id = 2;
%                                 elseif dest_loc_list(k) == agent_loc
%                                     src_id = agent_loc;
%                                     dest_id = agent_loc;
%                                 else
%                                     src_id = 2;
%                                     dest_id = 1;
%                                 end
%                                 obj.common_estimates{ii}.explicit_update(msg(k),type(1,k),src_id,dest_id);
% 
%                                 obj.common_estimates{ii}.msg_sent = obj.common_estimates{ii}.msg_sent+1;
%                                 obj.msgs_sent = obj.msgs_sent + 1;
% 
%                         else % implicit update
%                             ii = i;
% %                             if dest_loc_list(k) == agent_loc
% %                                 disp('DEAL WITH ME')
% %                             end
%                             
%                             conn_loc = find(sort([obj.agent_id,obj.connections]) == obj.common_estimates{i}.connection);
%                             
%                             if dest_loc_list(k) > agent_loc
%                                 x_local = [obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4); obj.local_filter.x(4*(dest_loc_list(k)-1)+1:4*(dest_loc_list(k)-1)+4)];
%                                 P_local = obj.local_filter.P([4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(dest_loc_list(k)-1)+1:4*(dest_loc_list(k)-1)+4],...
%                                                 [4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(dest_loc_list(k)-1)+1:4*(dest_loc_list(k)-1)+4]);
%                                 src_id = 1;
%                                 dest_id = 2;
%                             elseif dest_loc_list(k) == agent_loc
%                                 if agent_loc > conn_loc
%                                     x_local = [obj.local_filter.x(4*(conn_loc-1)+1:4*(conn_loc-1)+4); obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4)];
%                                     P_local = obj.local_filter.P([4*(conn_loc-1)+1:4*(conn_loc-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4],...
%                                                 [4*(conn_loc-1)+1:4*(conn_loc-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4]);
%                                     src_id = 2;
%                                     dest_id = 2;
%                                 else
%                                     x_local = [obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4); obj.local_filter.x(4*(conn_loc-1)+1:4*(conn_loc-1)+4)];
%                                     P_local = obj.local_filter.P([4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(conn_loc-1)+1:4*(conn_loc-1)+4],...
%                                                 [4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(conn_loc-1)+1:4*(conn_loc-1)+4]);
%                                     src_id = 1;
%                                     dest_id = 1;
%                                 end
%                             else
%                                 x_local = [obj.local_filter.x(4*(dest_loc_list(k)-1)+1:4*(dest_loc_list(k)-1)+4); obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4)];
%                                 P_local = obj.local_filter.P([4*(dest_loc_list(k)-1)+1:4*(dest_loc_list(k)-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4],...
%                                                 [4*(dest_loc_list(k)-1)+1:4*(dest_loc_list(k)-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4]);
%                                 src_id = 2;
%                                 dest_id = 1;
%                             end
%                                     
%                             obj.common_estimates{ii}.implicit_update(msg(k),type(k),src_id,dest_id,x_local,P_local);
% 
%                         end
%                     end
%                     
%                     % create outgoing msg struct
%                     outgoing_msgs{end+1} = struct('src',obj.agent_id,'dest',obj.common_estimates{i}.connection,...
%                         'status',{transmit},'type',{type},'data',{msg});
%                     
%                 end
%             end
%             if (sum(isnan(obj.local_filter.x)) > 0) || (sum(isinf(obj.local_filter.x)) > 0)
%                 disp('help!')
%             end
%         end
% 


        function process_received_measurements(obj,inbox)
            
            % process each collected measurement
            for i=1:length(inbox)
                if ~isempty(inbox{i})

                    % source of local measurements is agent
                    assert(inbox{i}.dest == obj.agent_id);

                    [~,src_idx] = obj.get_location(inbox{i}.src);
                    [~,dest_idx] = obj.get_location(inbox{i}.dest);

                    % update local filter
                    x_local = obj.local_filter.x;
                    P_local = obj.local_filter.P;
                    obj.local_filter.msg_update(inbox{i},x_local,P_local);
                    
                    x_local_comm = obj.local_filter.x(sort([src_idx,dest_idx]));
                    P_local_comm = obj.local_filter.P(sort([src_idx,dest_idx]),sort([src_idx,dest_idx]));

%                     obj.total_msgs = obj.total_msgs + 1;

                    % threshold measurement and update common estimate
                    for j=1:length(obj.common_estimates)

                        % find common estimate associated with msg destination
                        if obj.common_estimates{j}.connection == inbox{i}.src

                            % update common estimate with thresholding result
                            obj.common_estimates{j}.msg_update(inbox{i},x_local_comm,P_local_comm);

                        end
                    end 
                end
            end
        end






%         function process_received_measurements(obj,inbox)
%             
% %             agent_loc = find(obj.connections>obj.agent_id);
%             agent_loc = find(sort([obj.connections,obj.agent_id]) == obj.agent_id);
%             
%             % based on inbox measurements perform implicit and explicit
%             % updates
%             for i=1:length(inbox)
%                 if ~isempty(inbox{i})
%                     src = inbox{i}.src;
%                     dest = inbox{i}.dest;
%                     status = inbox{i}.status;
%                     type = inbox{i}.type;
%                     data = inbox{i}.data;
%                     
%                     % find location in states of measurement source
%                     src_loc = find(sort([obj.connections,obj.agent_id]) == src);
%                     if isempty(src_loc)
%                         src_loc = agent_loc;
%                     end
%                      
%                     % location of destination is self
%                     assert(dest == obj.agent_id);
%                     dest_loc = agent_loc;
%                     
%                     % loop through every element of each measurement
%                     for j=1:size(data,2)
%                         
%                         if (all(status(:,j))) && (binornd(1,obj.msg_success_prob)) % explicit update
%                             
%                             % perform local filter explicit update
%                             obj.local_filter.explicit_update(data(j),type(1,j),src_loc,dest_loc);
%                             
%                             if strcmp(type{1,j},'abs')
%                                 disp('HELP')
%                             end
%                             
%                             % find common est w/ source of measurement
%                             for k=1:length(obj.common_estimates)
%                                 if obj.common_estimates{k}.connection == src
%                                     
%                                     assert(src ~= obj.agent_id);
%                                     
%                                     if src_loc > agent_loc % agent_loc is the the dest_loc
%                                         src_loc_new = 2;
%                                         dest_loc_new = 1;
%                                     else
%                                         src_loc_new = 1;
%                                         dest_loc_new = 2;
%                                     end
%                                     
%                                     obj.common_estimates{k}.explicit_update(data(j),type(1,j),src_loc_new,dest_loc_new);
%                                 
%                                 end
%                             end
%                             
%                         else % implicit update
%                             
%                             % snapshot of local estimate for innovation
%                             local_est = obj.local_filter.x; local_cov = obj.local_filter.P;
%                             
%                             % local filter implicit measurement
%                             obj.local_filter.implicit_update(data(j),type(j),src_loc,dest_loc,local_est,local_cov);
% 
%                             % find common est w/ source of measurement
%                             for k=1:length(obj.common_estimates)
%                                 if obj.common_estimates{k}.connection == src
%                                     
%                                     % determine locations in common
%                                     % estimate and grab estimate snapshot
%                                     assert(src ~= obj.agent_id);
%                                     if src_loc > agent_loc
%                                         x_local = [obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4); obj.local_filter.x(4*(src_loc-1)+1:4*(src_loc-1)+4)];
%                                         P_local = obj.local_filter.P([4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(src_loc-1)+1:4*(src_loc-1)+4],...
%                                                         [4*(agent_loc-1)+1:4*(agent_loc-1)+4,4*(src_loc-1)+1:4*(src_loc-1)+4]);
%                                         src_loc = 2;
%                                         dest_loc = 1;
%                                         
%                                     else
%                                         x_local = [obj.local_filter.x(4*(src_loc-1)+1:4*(src_loc-1)+4); obj.local_filter.x(4*(agent_loc-1)+1:4*(agent_loc-1)+4)];
%                                         P_local = obj.local_filter.P([4*(src_loc-1)+1:4*(src_loc-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4],...
%                                                             [4*(src_loc-1)+1:4*(src_loc-1)+4,4*(agent_loc-1)+1:4*(agent_loc-1)+4]);
%                                         src_loc = 1;
%                                         dest_loc = 2;
%                                     end
%                                     
%                                     % perform implicit update w/ common est
%                                     obj.common_estimates{k}.implicit_update(data(j),type(j),src_loc,dest_loc,x_local,P_local);
%                                 end
%                             end
%                         end
%                     end
%                 end
%             end
%         end
    end
end