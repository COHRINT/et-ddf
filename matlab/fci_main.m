% N-agent linear event-triggered cooperative localization
% w/ reduced state vectors
%
% Ian Loefgren
% 1.9.2019
%
% 2D linear cooperative localization, different from n_agent_lin_coop_loc.m
% in that the state vectors of each agent are reduced from the whole
% network's states.

clear; close all; clc;

rng(200)

% number of agents
N = 6;
% connection topology: tree
num_connections = 3;

% event-triggering params
delta = 3;
tau = 20000;
msg_drop_prob = 0;

% simulation params
max_time = 20;
dt = 0.1;
input_tvec = 0:dt:max_time;

%% True starting position and input

for i=1:N
    x_true = [i*10,0,i*10,0]' + mvnrnd([0,0,0,0],5*eye(4))';
    x_true_vec((i-1)*4+1:(i-1)*4+4,1) = x_true;
    
    u((i-1)*2+1:(i-1)*2+2,:) = [2*cos(0.75*input_tvec);2*sin(input_tvec)];
end

%% Create centralized KF

R_abs = 9*eye(2);
R_rel = 25*eye(2);

[F_full,G_full] = ncv_dyn(dt,N);
Q_full = eye(4*N);
x0_full = x_true_vec;
P0_full = 100*eye(4*N);

baseline_filter = KF(F_full,G_full,0,0,Q_full,R_abs,R_rel,x_true_vec,P0_full,0);


%% Create agents objects

% specify connections
connections = {[3],[3],[1,2,4],[3,5,6],[4],[4]};

% for each platform, create dynamics models, and filters

agents = cell(N,1);
for i=1:N
    
    agent_id = i;
    
    % construct local estimates
    n = (length(connections{i})+1)*4;
    [F,G] = ncv_dyn(dt,length(connections{i})+1);
    Q_local = 0.1*eye(n);
    
    x0 = [x_true_vec((agent_id-1)*4+1:(agent_id-1)*4+4,1)];
    for j=1:length(connections{i})
        if connections{i}(j) > agent_id
            x0 = [x0; x_true_vec((j-1)*4+1:(j-1)*4+4,1)];
        else
            x0 = [x_true_vec((j-1)*4+1:(j-1)*4+4,1); x0];
        end
    end
    P0 = 100*eye(4*(length(connections{i})+1));
    
    local_filter = ETKF(F,G,0,0,Q_local,R_abs,R_rel,x0,P0,delta,agent_id,0);
    
    % construct common estimates, will always only have two agents
    [F_comm,G_comm] = ncv_dyn(dt,2);
    Q_comm = 0.1*eye(8);
    common_estimates = {};
    for j=1:length(connections{i})
        % make sure common estimate state vector is ordered by agent id
        if connections{i}(j) > agent_id
            x0_comm = [x_true_vec((agent_id-1)*4+1:(agent_id-1)*4+4,1); x_true_vec((j-1)*4+1:(j-1)*4+4,1)];
        else
            x0_comm = [x_true_vec((j-1)*4+1:(j-1)*4+4,1); x_true_vec((agent_id-1)*4+1:(agent_id-1)*4+4,1)];
%         x0_comm = [x_true_vec((agent_id-1)*4+1:(agent_id-1)*4+4,1); x_true_vec((j-1)*4+1:(j-1)*4+4,1)];
        end
        P0_comm = 100*eye(8);
        common_estimates{j} = ETKF(F_comm,G_comm,0,0,Q_comm,R_abs,R_rel,x0_comm,P0_comm,delta,agent_id,connections{i}(j));
    end
    
    agents{i} = Agent(agent_id,connections{i},local_filter,common_estimates,x_true_vec((i-1)*4+1:(i-1)*4+4,1),msg_drop_prob);
    
end

%% Main Simulation Loop

H_local = [1 0 0 0; 0 0 1 0];
H_rel = [1 0 0 0 -1 0 0 0; 0 0 1 0 0 0 -1 0];
[F_local,G_local] = ncv_dyn(dt);
Q_local = 0.1*eye(4);

ci_time_vec = zeros(N,length(input_tvec));

for i = 2:length(input_tvec)
%     tic
    clc;
    fprintf('Time step %i of %i, %f seconds of %f total\n',i,length(input_tvec),i*dt,length(input_tvec)*dt);
    
    % create measurement inbox
    inbox = cell(N,1);
    
    baseline_filter.predict(u(:,i));
    
    % process local measurements and determine which to send to connections
    for j=1:length(agents)
        msgs = {};
        
        % propagate true state
        w = mvnrnd([0,0,0,0],Q_local)';
        agents{j}.true_state(:,end+1) = F_local*agents{j}.true_state(:,end) + G_local*u(2*j-1:2*j,i) + w;
        
        % simulate measurements
        if ((j == 1) ||  (j == 6))
            v = mvnrnd([0,0],R_abs)';
            y_abs = H_local*agents{j}.true_state(:,end) + v;
%             fprintf('DRIVER LEVEL: Msg type, %s \t Msg src id, %i \t Msg dest id, %i \t Obj agent id, %i\n','abs',agents{j}.agent_id,agents{j}.agent_id,agents{j}.agent_id);
            y_abs_msg = struct('src',agents{j}.agent_id,'dest',agents{j}.agent_id,...
                        'status',[],'type',"abs",'data',y_abs);
            msgs = {y_abs_msg};
            
            baseline_filter.update(y_abs,'abs',agents{j}.agent_id,agents{j}.agent_id);
        end
        
        % relative position
%         rel_msgs = {};
        for k=1:length(agents{j}.connections)
            if agents{j}.connections(k) > 0
                v_rel = mvnrnd([0,0],R_rel)';
                y_rel = H_rel*[agents{j}.true_state(:,end); ...
                    agents{agents{j}.connections(k)}.true_state(:,end)] + v_rel;
%                 fprintf('DRIVER LEVEL: Msg type, %s \t Msg src id, %i \t Msg dest id, %i \t Obj agent id, %i\n','rel',agents{j}.agent_id,agents{j}.connections(k),agents{j}.agent_id);
                y_rel_msg = struct('src',agents{j}.agent_id,'dest',agents{j}.connections(k),...
                    'status',[],'type',"rel",'data',y_rel);
                msgs{end+1} = y_rel_msg;
                
                baseline_filter.update(y_rel,'rel',agents{j}.agent_id,agents{j}.connections(k));
            end
        end

        % update agent estimates
        outgoing = agents{j}.process_local_measurements(u(2*j-1:2*j,i),msgs);
        
%         add outgoing measurements to each agents "inbox"
        for k=1:length(outgoing)
            dest = outgoing{k}.dest;
            inbox{dest,j} = outgoing{k};
        end
    end
    
    % all agents now process received measurements, performing implizcit and
    % explicit measurement updates
    for j=1:length(agents)
        agents{j}.process_received_measurements({inbox{j,:}});
    end
    
    ci_inbox = cell(N,1);
    inbox_ind = 1;
    % covariance intersection between agents
    ci_trigger_list = zeros(1,length(agents));
    for j=1:length(agents)
        alpha = ones(4*(length(agents{j}.connections)+1),1);
        %           alpha = zeros(1,4*N);
        %           alpha(1:2:end) = 1;
        %           alpha(1) = 1; alpha(3) = 1; alpha(4*(N-1)+1) = 1; alpha(4*(N-1)+3) = 1;
        if trace(agents{j}.local_filter.P*diag(alpha)) > tau
            agents{j}.ci_trigger_cnt = agents{j}.ci_trigger_cnt + 1;
            ci_trigger_list(j) = 1;
            % save snapshot of state estimate and covariance, send to
            % connection inboxes
            x_snap = agents{j}.local_filter.x;
            P_snap = agents{j}.local_filter.P;
            for k=1:length(agents{j}.connections)
                ci_inbox{agents{j}.connections(k)}{end+1} = {x_snap,P_snap,j};
                x_conn_snap = agents{agents{j}.connections(k)}.local_filter.x;
                P_conn_snap = agents{agents{j}.connections(k)}.local_filter.P;
                ci_inbox{j}{end+1} = {x_conn_snap,P_conn_snap,agents{j}.connections(k)};
                
            end
        end
    end
    
    for j=1:length(agents)
        for k=1:length(ci_inbox{j})
            if ~isempty(ci_inbox{j}(k))
                xa = agents{j}.local_filter.x;
                Pa = agents{j}.local_filter.P;
                
                xb = ci_inbox{j}{k}{1};
                Pb = ci_inbox{j}{k}{2};
                alpha = ones(4*N,1);
                [xc,Pc] = covar_intersect(xa,xb,Pa,Pb,alpha);
                
                % update local estimates
                agents{j}.local_filter.x = xc;
                agents{j}.local_filter.P = Pc;
                
                %                   agents{agents{j}.connections(k)}.local_filter.x = xc;
                %                   agents{agents{j}.connections(k)}.local_filter.P = Pc;
                
                % update common estimates
                for ii=1:length(agents{j}.common_estimates)
                    if agents{j}.common_estimates{ii}.connection == ci_inbox{j}{k}{3}
                        agents{j}.common_estimates{ii}.x = xc;
                        agents{j}.common_estimates{ii}.P = Pc;
                    end
                end
                
%                 con1 = agents{j}.common_estimates{k}.connection;
%                 id1 = j;
%                 con2 = 0;
%                 id2 = 0;
%                 for ii=1:length(agents{agents{j}.connections(k)}.connections)
%                     if agents{agents{j}.connections(k)}.connections(ii) == j
%                         con2 = agents{agents{j}.connections(k)}.connections(ii);
%                         id2 = agents{j}.connections(k);
%                         agents{agents{j}.connections(k)}.common_estimates{ii}.x = xc;
%                         agents{agents{j}.connections(k)}.common_estimates{ii}.P = Pc;
%                         %                           fprintf('CI: common estimates agent id1: %i \t con1 %i \t agent id2 %i \t con2 %i\n',id1,con1,id2,con2);
%                     end
%                 end
                
                
            end
        end
    end
    
    % KEEP ME AROUND I'M A LITTLE BROKEN BUT NOT AS BAD AS CURRENT METHOD
%     for j=1:length(agents)
%         if ci_trigger_list(j)
%             for k=1:length(agents{j}.connections)
%                 xa = agents{j}.local_filter.x;
%                 Pa = agents{j}.local_filter.P;
%                 
%                 xb = agents{agents{j}.connections(k)}.local_filter.x;
%                 Pb = agents{agents{j}.connections(k)}.local_filter.P;
%                 
%                 [xc,Pc] = covar_intersect(xa,xb,Pa,Pb,alpha);
%                 
%                 % update local estimates
%                 agents{j}.local_filter.x = xc;
%                 agents{j}.local_filter.P = Pc;
%                 
%                 %                   agents{agents{j}.connections(k)}.local_filter.x = xc;
%                 %                   agents{agents{j}.connections(k)}.local_filter.P = Pc;
%                 
%                 % update common estimates
%                 agents{j}.common_estimates{k}.x = xc;
%                 agents{j}.common_estimates{k}.P = Pc;
%                 
%                 con1 = agents{j}.common_estimates{k}.connection;
%                 id1 = j;
%                 con2 = 0;
%                 id2 = 0;
%                 for ii=1:length(agents{agents{j}.connections(k)}.connections)
%                     if agents{agents{j}.connections(k)}.connections(ii) == j
%                         con2 = agents{agents{j}.connections(k)}.connections(ii);
%                         id2 = agents{j}.connections(k);
%                         agents{agents{j}.connections(k)}.common_estimates{ii}.x = xc;
%                         agents{agents{j}.connections(k)}.common_estimates{ii}.P = Pc;
%                         %                           fprintf('CI: common estimates agent id1: %i \t con1 %i \t agent id2 %i \t con2 %i\n',id1,con1,id2,con2);
%                     end
%                 end
%                 
%                 
%             end
%         end
%     end

    for j=1:length(agents)
        agents{j}.local_filter.state_history(:,i) = agents{j}.local_filter.x;
        agents{j}.local_filter.cov_history(:,:,i) = agents{j}.local_filter.P;
        
        for k=1:length(agents{j}.common_estimates)
            agents{j}.common_estimates{k}.state_history(:,i) = agents{j}.common_estimates{k}.x;
            agents{j}.common_estimates{k}.cov_history(:,:,i) = agents{j}.common_estimates{k}.P;
        end
    end
              
%     toc
end

