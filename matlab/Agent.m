%%% General "Agent" class for event-triggered cooperative localization

classdef Agent < handle
    properties
        agent_id
        connections
        local_filter
        common_estimates
        true_state
%         sensors
    end
    
    methods
        function obj = Agent(agent_id,connections,local_filter,common_estimates,x_true)
            
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
            
            % struct of sensors available to agent
%             obj.sensors = sensors;
        end
        
        function update(obj,input,local_measurements,received_measurements)
            
            % update local filter, and determine if local measurements
            % should be sent to connections
            outgoing = obj.process_local_measurements(input,local_measurements);
            
            % send measurements to connections
            obj.send_measurements(outgoing);
            
            % update common estimates with received measurements from
            % connections
            obj.process_recevied_measurements(received_measurements);
            
        end
        
        function [outgoing_msgs] = process_local_measurements(obj,input,local_measurements)
            
            % predict and update local filter
            obj.local_filter.predict(input);
            for i=1:length(local_measurements)
                if strcmp(local_measurements{i}.type,'abs')
                    obj.local_filter.update(local_measurements{i}.data);
                end
            end
            
            outgoing_msgs = {};
            
            % loop through common estimate filters and predict and update
            
%             outgoing = zeros(length(obj.common_estimates),size(local_measurements,1));
            
            for i=1:length(obj.common_estimates)
                
                % propagate common estimate
                obj.common_estimates{i}.predict(zeros(size(2*obj.common_estimates{1}.G,2),1));
                
                % for each connection, threshold local measurements
                for j=1:length(obj.common_estimates{i}.connection)
                    
                    [transmit,type,msg] = obj.common_estimates{i}.threshold(local_measurements); 
                    
                    % create outgoing msg struct
                    outgoing_msgs{end+1} = struct('src',obj.agent_id,'dest',obj.common_estimates{i}.connection,...
                        'status',{transmit},'type',{type},'data',{msg});
                end
            end
            
        end
        
        function process_received_measurements(obj,inbox)
            
            % based on inbox measurements perform implicit and explicit
            % updates
            for i=1:length(inbox)
                if ~isempty(inbox{i})
                    src = inbox{i}.src;
                    status = inbox{i}.status;
                    type = inbox{i}.type;
                    data = inbox{i}.data;
                    
                    for j=1:length(data)
                        if all(status(:,j)) % explicit update
                            for k=1:length(obj.common_estimates)
                                if obj.common_estimates{k}.connection == src
%                                     disp(type(1,j))
                                    obj.common_estimates{k}.explicit_update(data(j),type(1,j),src);
                                end
                            end
                        end
%                         else % implicit update
%                             for k=1:length(obj.connections)
%                                 if obj.common_estimates{k}.connection == src
%                                     obj.common_estimates{k}.implicit_update(data(j),type(j),src)
%                                 end
%                             end
%                         end
                    end
                end
                    

            
            end
        end
    end
end