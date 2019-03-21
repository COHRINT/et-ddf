%%% General "Agent" class for event-triggered cooperative localization

classdef Agent < handle
    properties
        agent_id
        connections
        meas_connections
        neighbor_connections
        num_states
        local_filter
        common_estimates
        true_state
        ci_trigger_cnt
        ci_trigger_rate
        msgs_sent
        total_msgs
        msg_success_prob
        tau_goal
        tau
        connection_tau_rates
        epsilon_1
        epsilon_2
        tau_history = []
        count
    end
    
    methods
        function obj = Agent(agent_id,connections,meas_connections,neighbor_connections,...
                                local_filter,common_estimates,x_true,msg_drop_prob,tau_goal,tau)
            
            % agent id
            obj.agent_id = agent_id;
            
            % ids of connections
            obj.connections = connections;
            obj.meas_connections = meas_connections;
            obj.neighbor_connections = neighbor_connections;
            
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
            obj.ci_trigger_rate = 0;
            
            obj.msgs_sent = 0;
            obj.total_msgs = 0;
            
            obj.msg_success_prob = 1-msg_drop_prob;
            
            % CI thresholding data
            obj.tau_goal = tau_goal;
            obj.tau = tau;
            obj.connection_tau_rates = zeros(length(obj.connections),1);
            obj.epsilon_1 = 0.05;
            obj.epsilon_2 = 0.1;

            % struct of sensors available to agent
%             obj.sensors = sensors;
        end
        
        function [loc,idx] = get_location(obj,id)
            % get location of agent in state estimate given id, and acutal
            % estimate indicies
            %
            % Input: scalar or vector of angent ids
            
            loc = [];
            idx = [];
            
            for i=1:length(id)  
                loc_new = find(sort([obj.agent_id,obj.connections]) == id(i));
                idx_new = obj.num_states*(loc_new-1)+1:obj.num_states*(loc_new-1)+obj.num_states;
                
                loc = [loc, loc_new];
                idx = [idx, idx_new];
            end
            
            loc = sort(loc);
            idx = sort(idx);
            
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
            for i=randperm(length(obj.common_estimates))
                % propagate common estimate, giving no input
                obj.common_estimates{i}.predict(zeros(size(obj.common_estimates{i}.G,2),1));
            end
            
            % create outgoing msg queue
            outgoing = {};
            loopback = {};
            
            % process each collected measurement
            for i=randperm(length(local_measurements))
                
                % unpack msg
                msg = local_measurements{i};
                src = msg.src;
                dest = msg.dest;
                target = msg.target;
                status = msg.status;
                type = msg.type;
                data = msg.data;

                % source of local measurements is agent
                assert(src == obj.agent_id);

                % update local filter
                obj.local_filter.msg_update(msg);
                
                % increase total possible msg count (+1 for each element)
%                 obj.total_msgs = obj.total_msgs + 2;
                
                % threshold measurement and update common estimate
                for j=randperm(length(obj.meas_connections))
                    
                    msg = local_measurements{i};
                    src = msg.src;
                    dest = msg.dest;
%                     status = msg.status;
%                     type = msg.type;
%                     data = msg.data;
                    
                    if ((dest == src) && (strcmp(type,"abs"))) || strcmp(type,"rel")
%                         dest = obj.common_estimates{j}.connection;
                        dest = obj.meas_connections(j);
                        
                        if isempty(msg.data)
                            disp('empty')
                        end
                        
                        msg = gen_msg(src,dest,target,status,type,data);
                        
                        if isempty(msg.data)
                            disp('empty data')
                        end
                    end
                    
                    % find common estimate associated with msg destination
                    for k=1:length(obj.common_estimates)
                    if obj.common_estimates{k}.meas_connection == msg.dest
                        
                        % threshold measurement
                        [src_,dest_,target_,status_,type_,data_] = obj.common_estimates{k}.threshold(msg);
                        
                        if isempty(status_)
                            disp('empty msg')
                        end
                        
                        [~,idx] = obj.get_location([obj.agent_id,obj.common_estimates{k}.connection]);
%                         [~,dest_idx] = obj.get_location(msg.dest);
                        
                        % generate msg from threshold results
                        msg = gen_msg(src_,dest_,target_,status_,type_,data_);
                        
                        % add measurement elements to be sent to sent cnt
                        obj.msgs_sent = obj.msgs_sent + sum(status_);
                        obj.total_msgs = obj.total_msgs + 2;
                        
                        % update common estimate with thresholding result
                        x_local = obj.local_filter.x(idx);
                        P_local = obj.local_filter.P(idx,idx);
%                         obj.common_estimates{j}.msg_update(msg,x_local,P_local);
                        
                        % add msg to outgoing msg queue
                        outgoing{end+1} = msg;
                        loopback{end+1} = {msg,x_local,P_local};
                        
                    end
                    end
                end              
            end
            
            for i=randperm(length(loopback))
                
                msg = loopback{i}{1};
                x_local = loopback{i}{2};
                P_local = loopback{i}{3};
                
                for j=randperm(length(obj.common_estimates))
                    
                    % find common estimate associated with msg destination
%                     if any(obj.common_estimates{j}.connection == msg.dest)
                    if obj.common_estimates{j}.meas_connection == msg.dest
                        
                        % update common estimate with thresholding result
                        obj.common_estimates{j}.msg_update(msg,x_local,P_local);
                        
                    end
                end
            end        
        end

        function process_received_measurements(obj,inbox)
            
            % process each collected measurement
            for i=randperm(length(inbox))
                if ~isempty(inbox{i})

                    % source of local measurements is agent
                    assert(inbox{i}.dest == obj.agent_id);

                    [~,src_idx] = obj.get_location(inbox{i}.src);
                    [~,dest_idx] = obj.get_location(inbox{i}.dest);
                    [~,target_idx] = obj.get_location(inbox{i}.target);
                    
                    % communication failure simulation
                    % draws array from binomial w/ msg success prob and
                    % bitmasks msg status to simulate that element was not
                    % sent
                    inbox{i}.status = inbox{i}.status & ...
                        binornd(1,obj.msg_success_prob,size(inbox{i}.status,1),size(inbox{i}.status,2));

                    % update local filter
                    x_local = obj.local_filter.x;
                    P_local = obj.local_filter.P;
                    obj.local_filter.msg_update(inbox{i},x_local,P_local);
                    

%                     obj.total_msgs = obj.total_msgs + 1;

                    % threshold measurement and update common estimate
                    for j=randperm(length(obj.common_estimates))
                        
                        [~,idx] = obj.get_location([obj.agent_id,obj.common_estimates{j}.connection]);
                        
%                         x_local_comm = obj.local_filter.x(sort([src_idx,dest_idx]));
%                         P_local_comm = obj.local_filter.P(sort([src_idx,dest_idx]),sort([src_idx,dest_idx]));
                        x_local_comm = obj.local_filter.x(idx);
                        P_local_comm = obj.local_filter.P(idx,idx);

                        % find common estimate associated with msg destination
%                         if obj.common_estimates{j}.connection == inbox{i}.src
                        if obj.common_estimates{j}.meas_connection == inbox{i}.src

                            % update common estimate with thresholding result
                            obj.common_estimates{j}.msg_update(inbox{i},x_local_comm,P_local_comm);

                        end
                    end 
                end
            end
        end
    end
end