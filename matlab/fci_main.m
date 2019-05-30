% N-agent linear event-triggered cooperative localization
% w/ reduced state vectors
%
% Ian Loefgren
% 1.9.2019
%
% 2D linear cooperative localization, different from n_agent_lin_coop_loc.m
% in that the state vectors of each agent are reduced from the whole
% network's states.

clear; %close all; clc;
% load('sensor_data_6agents.mat');

% rng(999)

%% Specify connections

% connections = {[3],[3],[1,2,4],[3,5,6],[4],[4]};

% connections = {[2],[1,3],[2,4],[3,5],[4,6],[5]};

% connections = {[5],[5],[6],[6],[1,2,7],[3,4,7],[5,6,8],[7,9,10],[8,11,12],...
%                 [8,13,14],[9],[9],[10],[10]};

connections = {[9],[9],[10],[10],[11],[11],[12],[12],...
                [1,2,13],[3,4,13],[5,6,14],[7,8,14],[9,10,15],[11,12,15],[13,14,16],...
                [15,17,18],[16,19,20],[16,21,22],[17,23,24],[17,25,26],[18,27,28],...
                [18,29,30],[19],[19],[20],[20],[21],[21],[22],[22]};

% connections = {[2,3,4,5],[1,3,6,7],[1,2,8,9],[1],[1],[2],[2],[3],[3]};

% connections = {[2],[1]};
            
% specify which platforms get gps-like measurements
abs_meas_vec = [13 14 17 18];

% number of agents
N = length(connections);
% connection topology: tree
num_connections = 2;

% delta_vec = 0:0.5:5;
% tau_state_goal_vec = 5:0.5:15;
% tau_state_vec = 0:0.5:25;

delta_vec = [2];
tau_state_goal_vec = [2,3.5,5,7,10];
msg_drop_prob_vec = 0;
num_mc_sims = 100;

% cost = zeros(length(delta_vec),length(tau_state_goal_vec),5);
w1 = 0.5;
w2 = 0.5;

loop_cnt = 1;
total_loop_time = 0;
avg_loop_time = 0;

max_time = 20;
dt = 0.1;
input_tvec = 0:dt:max_time;

total_sims = length(delta_vec)*length(tau_state_goal_vec)*length(msg_drop_prob_vec)*num_mc_sims;

mse_data = zeros(total_sims,4+length(input_tvec),N);
mse_rel_data = zeros(total_sims,4+length(input_tvec),N,N);
baseline_mse_data = zeros(total_sims,4+length(input_tvec),N);
baseline_mse_rel_data = zeros(total_sims,4+length(input_tvec),N,N);
msg_data = zeros(total_sims,4+N^2);
msg_rate_data = zeros(total_sims,4+N^2);
ci_data = zeros(total_sims,4+N);
ci_rate_data = zeros(total_sims,4+N);

cost = zeros(length(delta_vec)*length(tau_state_goal_vec)*length(msg_drop_prob_vec),9);
network_mse = zeros(N,length(input_tvec),length(msg_drop_prob_vec));
baseline_mse = zeros(N,length(input_tvec),length(delta_vec));

worst_ci_process_times = zeros(1,N);

for idx1=1:length(delta_vec)
for idx2=1:length(tau_state_goal_vec) 
for idx3=1:length(msg_drop_prob_vec)
for idx4=1:num_mc_sims
    
% start timer for loop time
tic;

% event-triggering  and covariance intersection params
% delta = 3;
delta = delta_vec(idx1);
% tau_goal = 100;
% tau = 70;
% tau_state_goal = 12.5;
tau_state_goal = tau_state_goal_vec(idx2);
% tau_state = 8.75;
% tau_state = tau_state_vec(idx3);
tau_state = 0.75*tau_state_goal;

use_adaptive = true;

% comms modeling params
msg_drop_prob = msg_drop_prob_vec(idx3);

mc_sim = idx4;

% simulation params
% max_time = 20;
% dt = 0.1;
% input_tvec = 0:dt:max_time;

%% True starting position and input

for i=1:N
    start_noise = mvnrnd([0,0,0,0],diag([5 0.5 5 0.5]))';
%     start_noise = x_start_data{i};
    x_true = [i*10,0,i*10,0]' + start_noise;
    x_true_vec((i-1)*4+1:(i-1)*4+4,1) = x_true;
    
    % generate input for platforms
    u((i-1)*2+1:(i-1)*2+2,:) = [2*cos(0.75*input_tvec);2*sin(0.75*input_tvec)];
%     u((i-1)*2+1:(i-1)*2+2,:) = [0.05*input_tvec;0.5*input_tvec];
end

%% Create centralized KF

Q_local_true = [0.0003 0.005 0 0;
            0.005 0.1 0 0;
            0 0 0.0003 0.005;
            0 0 0.005 0.1];
        
Q_local = [0.0017 0.025 0 0;
            0.025 0.5 0 0;
            0 0 0.0017 0.025;
            0 0 0.025 0.5];

% R_abs = 1*eye(2);
R_abs = diag([1 1]);
R_rel = 3*eye(2);

% generate dynamics matrices for baseline filter
[F_full,G_full] = ncv_dyn(dt,N);
% Q_full = 1*eye(4*N);
Q_full_cell = cell(1,N);
[Q_full_cell{:}] = deal(Q_local);
Q_full = blkdiag(Q_full_cell{:});
x0_full = x_true_vec;
P0_full = 100*eye(4*N);

% create baseline filter for comparison
baseline_filter = KF(F_full,G_full,0,0,Q_full,R_abs,R_rel,x_true_vec,P0_full,0);


%% Create agents objects

% for each platform, create dynamics models, and filters

agents = cell(N,1);
ci_trigger_mat = zeros(N,length(input_tvec));
for i=1:N
    
    agent_id = i;
    
    ids = sort([agent_id,connections{i}]);
    % add shortest path to gps node to estimate
%     gps_sp_ids = setdiff(shortest_paths{i},ids);
    neighbor_conn_ids = [];
    for j=1:length(connections{i})
        for k=1:length(connections{connections{i}(j)})
            if ~any(neighbor_conn_ids == connections{connections{i}(j)}(k))
                neighbor_conn_ids = [neighbor_conn_ids,connections{connections{i}(j)}];
            end
        end
    end
    neighbor_conn_ids = neighbor_conn_ids(neighbor_conn_ids~=agent_id);
%     ids = sort([ids,gps_sp_ids]);
    ids = sort([ids,neighbor_conn_ids]);
%     connections_new = sort([gps_sp_ids,connections{i}]);
    connections_new = sort([neighbor_conn_ids,connections{i}]);
    meas_connections = connections{i};
    
    est_state_length = length(ids);
    
    % construct local estimates
    n = (est_state_length)*4;
    [F,G] = ncv_dyn(dt,est_state_length);
%     Q_localfilter = 1*eye(n);
    Q_localfilter_cell = cell(1,est_state_length);
    
%     % construct local estimates
%     n = (length(connections{i})+1)*4;
%     [F,G] = ncv_dyn(dt,length(connections{i})+1);
% %     Q_localfilter = 1*eye(n);
%     Q_localfilter_cell = cell(1,length(connections{i})+1);
    [Q_localfilter_cell{:}] = deal(Q_local);
    Q_localfilter = blkdiag(Q_localfilter_cell{:});

    ids = sort([agent_id,connections_new]);
    x0 = [];
    for j=1:length(ids)
        x0 = [x0; x_true_vec((ids(j)-1)*4+1:(ids(j)-1)*4+4,1)];
    end
    
    P0 = 100*eye(4*est_state_length);
    
    local_filter = ETKF(F,G,0,0,Q_localfilter,R_abs,R_rel,x0,P0,delta,agent_id,connections_new,-1);
    
    % construct common estimates, with intersection of states
    % loop over meas_connections, aka direct connections
    common_estimates = {};
    for j=1:length(meas_connections)
        
        % find intersection of states
%         inter_states = intersect([agent_id,connections_new],[connections{i}(j),connections{connections{i}(j)}]); 
        inter_states = unique([meas_connections,connections{connections{i}(j)}]);
        
        % make sure common estimate state vector is ordered by agent id
%         comm_ids = sort([agent_id,connections{i}(j)]);
        comm_ids = inter_states;
        x0_comm = [];
        
        for k=1:length(comm_ids)
            x0_comm = [x0_comm; x_true_vec((comm_ids(k)-1)*4+1:(comm_ids(k)-1)*4+4,1)];
        end

        P0_comm = 100*eye(4*length(comm_ids));
        
        [F_comm,G_comm] = ncv_dyn(dt,length(comm_ids));
%         Q_comm = 1*eye(8);
        Q_comm_cell = cell(1,length(comm_ids));
        [Q_comm_cell{:}] = deal(Q_local);
        Q_comm = blkdiag(Q_comm_cell{:});
        
%         common_estimates = {};
        comm_ids = comm_ids(comm_ids~=agent_id);
    
        common_estimates{j} = ETKF(F_comm,G_comm,0,0,Q_comm,R_abs,R_rel,x0_comm,P0_comm,delta,agent_id,comm_ids,meas_connections(j));
    end
    
    agents{i} = Agent(agent_id,connections_new,meas_connections,neighbor_conn_ids,...
                        local_filter,common_estimates,x_true_vec((i-1)*4+1:(i-1)*4+4,1),...
                        msg_drop_prob,length(x0)*tau_state_goal,length(x0)*tau_state);
    
end

%% Main Simulation Loop

H_local = [1 0 0 0; 0 0 1 0];
H_rel = [1 0 0 0 -1 0 0 0; 0 0 1 0 0 0 -1 0];
[F_local,G_local] = ncv_dyn(dt);
% Q_local = 0.1*eye(4);


ci_time_vec = zeros(N,length(input_tvec));
all_msgs = {};
abs_meas_mat = zeros(N,length(input_tvec),2);
rel_meas_mat = zeros(N,length(input_tvec),2);

mse_data(loop_cnt,1,:) = delta;
mse_data(loop_cnt,2,:) = tau_state_goal;
mse_data(loop_cnt,3,:) = msg_drop_prob;
mse_data(loop_cnt,4,:) = mc_sim;

mse_rel_data(loop_cnt,1,:,:) = delta;
mse_rel_data(loop_cnt,2,:,:) = tau_state_goal;
mse_rel_data(loop_cnt,3,:,:) = msg_drop_prob;
mse_rel_data(loop_cnt,4,:,:) = mc_sim;

baseline_mse_data(loop_cnt,1,:) = delta;
baseline_mse_data(loop_cnt,2,:) = tau_state_goal;
baseline_mse_data(loop_cnt,3,:) = msg_drop_prob;
baseline_mse_data(loop_cnt,4,:) = mc_sim;

baseline_mse_rel_data(loop_cnt,1,:,:) = delta;
baseline_mse_rel_data(loop_cnt,2,:,:) = tau_state_goal;
baseline_mse_rel_data(loop_cnt,3,:,:) = msg_drop_prob;
baseline_mse_rel_data(loop_cnt,4,:,:) = mc_sim;

for i = 2:length(input_tvec)
%     tic

%     for j=1:length(agents)
%         fprintf('agent %i est:\n',j)
%         disp(agents{j}.local_filter.x)
%     end

    clc;
    fprintf('Iteration %i of %i\n',loop_cnt,length(delta_vec)*length(tau_state_goal_vec)*length(msg_drop_prob_vec)*num_mc_sims);
    fprintf('Delta: %f \t State tau goal: %f\n',delta,tau_state_goal);
    fprintf('Monte Carlo Sim %i of %i\n',idx4,num_mc_sims);
    fprintf('Time step %i of %i, %f seconds of %f total\n',i,length(input_tvec),i*dt,length(input_tvec)*dt);
    
    % compute approx time to go
    iterations_left = length(delta_vec)*length(tau_state_goal_vec)*length(msg_drop_prob_vec)*num_mc_sims - loop_cnt + 1;
    est_time_remaining_hour = floor((iterations_left * avg_loop_time)/3600);
    est_time_remaining_min = floor(mod((iterations_left * avg_loop_time)/60,60));
    est_time_remaining_sec = floor(mod((iterations_left* avg_loop_time),60));
    
    fprintf('Average iteration time: %0.2f seconds\n',avg_loop_time);
    fprintf('Estimated time remaining: %i hr, %i min, %i sec\n',est_time_remaining_hour,est_time_remaining_min,est_time_remaining_sec);
    
    % create measurement inbox
    inbox = cell(N,1);
    
    baseline_filter.predict(u(:,i-1));
%     disp(u(:,i-1))
%     baseline_filter.predict(zeros(size(u(:,i))));
    
    % process local measurements and determine which to send to connections
%     for j=randperm(length(agents))
    for j=1:length(agents)
        msgs = {};
        
%         if i == 107 && j == 16
%             disp('break')
%         end
        
        % propagate true state
        w = mvnrnd([0,0,0,0],Q_local_true)';
%         w = w_data{j}(:,i-1);
        agents{j}.true_state(:,end+1) = F_local*agents{j}.true_state(:,end) + G_local*u(2*(j-1)+1:2*(j-1)+2,i-1) + w;
        
        % simulate measurements: absolute measurements
        if ismember(agents{j}.agent_id,abs_meas_vec)
            v = mvnrnd([0,0],R_abs)';
%             v = v_data{j}(:,i-1);
            y_abs = H_local*agents{j}.true_state(:,end) + v;
            y_abs_msg = struct('src',agents{j}.agent_id,'dest',agents{j}.agent_id,...
                        'target',agents{j}.agent_id,'status',[1 1],'type',"abs",'data',y_abs);
            msgs = {y_abs_msg};
            
            if binornd(1,1-msg_drop_prob)
            baseline_filter.update(y_abs,'abs',agents{j}.agent_id,agents{j}.agent_id);
            end
            
            abs_meas_mat(j,i,1) = y_abs(1);
            abs_meas_mat(j,i,2) = y_abs(2);
        end
        
        % relative position
%         for k=randperm(length(agents{j}.meas_connections))
        for k=1:length(agents{j}.meas_connections)
%             if agents{j}.agent_id ~= 5
                v_rel = mvnrnd([0,0],R_rel)';
%                 v_rel = v_rel_data{j}(:,i-1);
                y_rel = H_rel*[agents{j}.true_state(:,end); ...
                    agents{agents{j}.meas_connections(k)}.true_state(:,end)] + v_rel;
                y_rel_msg = struct('src',agents{j}.agent_id,'dest',agents{j}.meas_connections(k),...
                    'target',agents{j}.meas_connections(k),'status',[1 1],'type',"rel",'data',y_rel);
                msgs{end+1} = y_rel_msg;
                
                if binornd(1,1-msg_drop_prob)
                baseline_filter.update(y_rel,'rel',agents{j}.agent_id,agents{j}.meas_connections(k));
                end
                
                rel_meas_mat(j,i,1) = y_rel(1);
                rel_meas_mat(j,i,2) = y_rel(2);
%             end
        end

        %% Process the generated measurements locally, determine which to send
        input = u(2*(j-1)+1:2*(j-1)+2,i);
%         input = zeros(2,1);
        outgoing = agents{j}.process_local_measurements(input,msgs);
        
%         add outgoing measurements to each agents "inbox"
%         for k=randperm(length(outgoing))
        for k=1:length(outgoing)
            dest = outgoing{k}.dest;
            inbox{dest,end+1} = outgoing{k};
            all_msgs{end+1} = outgoing{k};
        end
    end
    
    %% All agents now process received measurements, performing implicit and
    % explicit measurement updates
%     for j=randperm(length(agents))
    for j=1:length(agents)
        if j == 5 && i == 100
            disp('break')
        end
        agents{j}.process_received_measurements({inbox{j,:}});
    end
    
    %% Covariance intersection thresholding and snapshotting
    ci_inbox = cell(N,1);
    inbox_ind = 1;
    % covariance intersection between agents
    ci_trigger_list = zeros(1,length(agents));
    
%     for j=randperm(length(agents))
    for j=1:length(agents)
        alpha = ones(4*(length(agents{j}.connections)+1),1);

        % check trace of cov to determine if CI should be triggered
        if trace(agents{j}.local_filter.P*diag(alpha)) > agents{j}.tau || i<10
            agents{j}.ci_trigger_cnt = agents{j}.ci_trigger_cnt + 1;
            agents{j}.ci_trigger_rate = agents{j}.ci_trigger_cnt / (i-1);
            ci_trigger_list(j) = 1;
            ci_trigger_mat(j,i) = 1;
            
            % determine common states and grab snapshot of those common
            % states
            % save snapshot of state estimate and covariance, send to
            % connection inboxes
            x_snap = agents{j}.local_filter.x;
            P_snap = agents{j}.local_filter.P;
%             for k=randperm(length(agents{j}.meas_connections))
            for k=1:length(agents{j}.meas_connections)
    
                % compute transforms for platform and connection, and
                % number of intersecting states
                
                [Ta,il_a] = gen_sim_transform(agents{j}.agent_id,agents{j}.connections,...
                    agents{agents{j}.meas_connections(k)}.agent_id,agents{agents{j}.meas_connections(k)}.connections);
                [Tb,il_b] = gen_sim_transform(agents{agents{j}.meas_connections(k)}.agent_id,...
                    agents{agents{j}.meas_connections(k)}.connections,agents{j}.agent_id,agents{j}.connections);

                % transform means and covariances to group common states at
                % beginning of state vector/covariance
                xaT = Ta\x_snap;
                xaTred = xaT(1:il_a);
                PaT = Ta\P_snap*Ta;
                PaTred = PaT(1:il_a,1:il_a);
                
%                 ci_inbox{agents{j}.connections(k)}{end+1} = {x_snap,P_snap,agents{j}.agent_id,agents{j}.connections};
                ci_inbox{agents{j}.meas_connections(k)}{end+1} = {xaTred,PaTred,agents{j}.agent_id,agents{j}.connections,agents{j}.tau};
                
                x_conn_snap = agents{agents{j}.meas_connections(k)}.local_filter.x;
                P_conn_snap = agents{agents{j}.meas_connections(k)}.local_filter.P;
                
                xbT = Tb\x_conn_snap;
                xbTred = xbT(1:il_b);
                PbT = Tb\P_conn_snap*Tb;
                PbTred = PbT(1:il_b,1:il_b);
                
%                 ci_inbox{j}{end+1} = {x_conn_snap,P_conn_snap,agents{j}.connections(k),agents{agents{j}.connections(k)}.connections};
                ci_inbox{j}{end+1} = {xbTred,PbTred,agents{j}.meas_connections(k),agents{agents{j}.meas_connections(k)}.connections,agents{agents{j}.meas_connections(k)}.ci_trigger_rate};
                
                if isempty(agents{agents{j}.meas_connections(k)}.connections)
                    disp('break')
                end
                
%                 disp(ci_inbox{j}{end}{4})
                
            end
        end
    end
    
    %% Acutal covariance intersection performed (w/ conditional updates on full states)
%     for j=randperm(length(agents))
    for j=1:length(agents)
        
%         if ((i > 100 && j == 16) && abs(agents{j}.local_filter.x(5) - agents{j}.true_state(1,end)) > 4)
%             disp('break')
%         end
        
%         if i == 106 && j == 16
%             disp('break')
%         end
        
%         for k=randperm(length(ci_inbox{j}))
        for k=1:length(ci_inbox{j})
            if ~isempty(ci_inbox{j}{k})
                
%                 tic;
                
                xa = agents{j}.local_filter.x;
                Pa = agents{j}.local_filter.P;
                
                xb = ci_inbox{j}{k}{1};
                Pb = ci_inbox{j}{k}{2};
                b_id = ci_inbox{j}{k}{3};
                b_connections = ci_inbox{j}{k}{4};
                b_rate = ci_inbox{j}{k}{5};
                
                % construct transformation
                [Ta,il_a,inter] = gen_sim_transform(agents{j}.agent_id,agents{j}.connections,b_id,b_connections);
                
                % transform means and covariances to group common states at
                % beginning of state vector/covariance
                xaT = Ta\xa;
                xaTred = xaT(1:il_a);
                PaT = Ta\Pa*Ta;
                PaTred = PaT(1:il_a,1:il_a);
                
                xbTred = xb;
                PbTred = Pb;

                alpha = ones(size(PaTred,1),1);
                [xc,Pc] = covar_intersect(xaTred,xbTred,PaTred,PbTred,alpha);
                
                % compute information delta for conditional update
                invD = inv(Pc) - inv(PaTred);
                invDd = Pc\xc - PaTred\xaTred;
                
                % conditional gaussian update
                V = inv(inv(PaT) + [invD zeros(size(Pc,1),size(PaT,2)-size(Pc,2)); ...
                    zeros(size(PaT,1)-size(Pc,1),size(Pc,2)) zeros(size(PaT)-size(Pc))]);
                v = V*(PaT\xaT + [invDd; zeros(size(PaT,1)-size(Pc,1),1)]);
                
                % transform back to normal state order
                xa = Ta*v;
                Pa = Ta*V/Ta;
                
                if i==100
                    bp = 1;
                end
                
                % update local estimates
                agents{j}.local_filter.x = xa;
                agents{j}.local_filter.P = Pa;
                
                % update common estimates
%                 for ii=randperm(length(agents{j}.common_estimates))
                for ii=1:length(agents{j}.common_estimates)
                    if agents{j}.common_estimates{ii}.meas_connection == ci_inbox{j}{k}{3}
%                         conn_loc = inter==agents{j}.common_estimates{ii}.connection;
                        agents{j}.common_estimates{ii}.x = xc;
                        agents{j}.common_estimates{ii}.P = Pc;
                        
                        agents{j}.connection_tau_rates(ii) = b_rate;
                    end
                end
                
%                 time_elapsed = toc;
%                 if time_elapsed > worst_ci_process_times(j)
%                     worst_ci_process_times(j) = time_elapsed;
%                 end
                
            end
        end
        
        % update CI threshold if using adaptive method
        if use_adaptive
            agents{j}.tau = min(agents{j}.tau_goal,agents{j}.tau + ...
                        agents{j}.epsilon_1*sum(-agents{j}.connection_tau_rates+agents{j}.ci_trigger_rate*ones(length(agents{j}.connection_tau_rates),1)) + ...
                        agents{j}.epsilon_2*(agents{j}.tau_goal-agents{j}.tau));
        end
    end

    %% Update state history of each agent (for plotting)
    for j=1:length(agents)
        agents{j}.local_filter.state_history(:,i) = agents{j}.local_filter.x;
        agents{j}.local_filter.cov_history(:,:,i) = agents{j}.local_filter.P;
        agents{j}.tau_history(i) = agents{j}.tau;
        
        for k=1:length(agents{j}.common_estimates)
            agents{j}.common_estimates{k}.state_history(:,i) = agents{j}.common_estimates{k}.x;
            agents{j}.common_estimates{k}.cov_history(:,:,i) = agents{j}.common_estimates{k}.P;
        end
        
        [loc,iidx] = agents{j}.get_location(agents{j}.agent_id);
%         network_mse(j,i,idx1) = sum((agents{j}.local_filter.state_history(iidx,i) - agents{j}.true_state(:,i)).^2,1)./4;
        network_mse(j,i,idx2) = norm(agents{j}.local_filter.state_history([iidx(1),iidx(3)],i) - agents{j}.true_state([1 3],i))^2;
        
        mse_data(loop_cnt,4+i,j) = norm(agents{j}.local_filter.state_history([iidx(1),iidx(3)],i) - agents{j}.true_state([1 3],i))^2;
        
        baseline_mse(j,i,idx2) = norm(baseline_filter.state_history([4*(j-1)+1,4*(j-1)+3],i) - agents{j}.true_state([1 3],i))^2;
        
        baseline_mse_data(loop_cnt,4+i,j) = norm(baseline_filter.state_history([4*(j-1)+1,4*(j-1)+3],i) - agents{j}.true_state([1 3],i))^2;
        
        for k=1:length(agents{j}.common_estimates)
            [rel_loc,rel_iidx] = agents{j}.get_location(agents{j}.meas_connections(k));
            rel_network_mse(j,agents{j}.meas_connections(k),i,idx2) = ...
                        norm( (agents{j}.local_filter.state_history([iidx(1),iidx(3)],i) -...
                        agents{j}.local_filter.state_history([rel_iidx(1),rel_iidx(3)],i)) - ...
                        (agents{j}.true_state([1 3],i) - agents{agents{j}.meas_connections(k)}.true_state([1 3],i)) )^2;
                    
            mse_rel_data(loop_cnt,4+i,j,agents{j}.meas_connections(k)) = ...
                        norm( (agents{j}.local_filter.state_history([iidx(1),iidx(3)],i) -...
                        agents{j}.local_filter.state_history([rel_iidx(1),rel_iidx(3)],i)) - ...
                        (agents{j}.true_state([1 3],i) - agents{agents{j}.meas_connections(k)}.true_state([1 3],i)) )^2;
            
            m = agents{j}.meas_connections(k);
            rel_baseline_mse(j,agents{j}.meas_connections(k),i,idx2) = ...
                        norm( (baseline_filter.state_history([4*(j-1)+1,4*(j-1)+3],i) -...
                        baseline_filter.state_history([4*(m-1)+1,4*(m-1)+3],i)) - ...
                        (agents{j}.true_state([1 3],i) - agents{m}.true_state([1 3],i)) )^2;
                    
            baseline_mse_rel_data(loop_cnt,4+i,j,agents{j}.meas_connections(k)) = ...
                        norm( (baseline_filter.state_history([4*(j-1)+1,4*(j-1)+3],i) -...
                        baseline_filter.state_history([4*(m-1)+1,4*(m-1)+3],i)) - ...
                        (agents{j}.true_state([1 3],i) - agents{m}.true_state([1 3],i)) )^2;
        end
        
        
        
    end
              
%     toc


end

% for iii=1:N
%     fprintf('Agent %i worst time: %f\n',iii,worst_ci_process_times(iii))
% end

avg_mse = mean(network_mse,1);

%% compute costs and FOMs

% compute average covariance trace
% est_err_vec = zeros(1,N);
covar_mean_vec = zeros(1,N);
for jj=1:length(agents)
%     err_vec = zeros(1,size(agents{jj}.local_filter.state_history,2));
    trace_vec = zeros(1,size(agents{jj}.local_filter.cov_history,3));
    for kk=1:size(agents{jj}.local_filter.cov_history,3)
        trace_vec(kk) = trace(agents{jj}.local_filter.cov_history(:,:,kk));
    end
    covar_mean_vec(jj) = mean(trace_vec);
end
% est_err_avg = mean(est_err_vec);
covar_avg = mean(covar_mean_vec);

% compute total and average data transfer

%  dim1=src agent, dim2=dest agent, dim3=meas type, dim4=element [x or y]
comms_mat_sent = zeros(N,N,2,2);
comms_mat_total = zeros(N,N,2,2);
for jj=1:length(all_msgs)
    msg = all_msgs{jj};
    
    type = msg.type=="rel";
    
    for kk=1:length(msg.status)
        comms_mat_total(msg.src,msg.dest,type+1,kk) = comms_mat_total(msg.src,msg.dest,type+1,kk) + 1;
        if msg.status(kk)
            comms_mat_sent(msg.src,msg.dest,type+1,kk) = comms_mat_sent(msg.src,msg.dest,type+1,kk) + 1;
        end
    end  
end

ci_trigger_vec = zeros(1,N);
usage_vec_ci = zeros(1,N);

for jj=1:length(agents)
    ci_trigger_vec(1,jj) = agents{jj}.ci_trigger_cnt;
%     usage_vec_ci(1,i) = agents{i}.ci_trigger_cnt * (size(agents{i}.local_filter.x,1)^2 + size(agents{i}.local_filter.x,1)) * length(agents{i}.connections);
    usage_vec_ci(1,jj) = agents{jj}.ci_trigger_cnt * (72) * length(agents{jj}.connections);
end

usage_vec_msg = sum(comms_mat_sent(:,:,1,1),2)' + sum(comms_mat_sent(:,:,1,2),2)' + sum(comms_mat_sent(:,:,2,1),2)' + sum(comms_mat_sent(:,:,1,2),2)';
usage_vec = usage_vec_ci + usage_vec_msg;

data_trans_avg = mean(usage_vec);

% compute cost fxn
cost_val = w1*(covar_avg/max(covar_mean_vec)) + w2*(data_trans_avg/max(usage_vec));

% compute estimation error average
err_vec = zeros(1,N);
for jj=1:length(agents)
    
    % get location
    [loc,iidx] = agents{jj}.get_location(agents{jj}.agent_id);
    % compute average est err per agent
    err_vec(jj) = mean(mean((agents{jj}.local_filter.state_history(iidx,:) - agents{jj}.true_state(:,:)),2));
end
est_err = mean(err_vec);

for jj=1:length(agents)
    [loc,iidx] = agents{jj}.get_location(agents{jj}.agent_id);
    err_vec(jj) = mean(sqrt(sum((agents{jj}.local_filter.state_history(iidx,:) - agents{jj}.true_state(:,:)).^2,2)./length(input_tvec)));
end
est_rmse = mean(err_vec);

cost(loop_cnt,:) = [loop_cnt delta tau_state_goal covar_avg msg_drop_prob data_trans_avg cost_val est_err est_rmse];

% cost(idx,3) = covar_avg;
% cost(idx,4) = data_trans_avg;
% cost(idx,5) = cost_val;

msg_data(loop_cnt,1,:) = delta;
msg_data(loop_cnt,2,:) = tau_state_goal;
msg_data(loop_cnt,3,:) = msg_drop_prob;
msg_data(loop_cnt,4,:) = mc_sim;

msg_rate_data(loop_cnt,1,:) = delta;
msg_rate_data(loop_cnt,2,:) = tau_state_goal;
msg_rate_data(loop_cnt,3,:) = msg_drop_prob;
msg_rate_data(loop_cnt,4,:) = mc_sim;
    
ci_data(loop_cnt,1,:) = delta;
ci_data(loop_cnt,2,:) = tau_state_goal;
ci_data(loop_cnt,3,:) = msg_drop_prob;
ci_data(loop_cnt,4,:) = mc_sim;

ci_rate_data(loop_cnt,1,:) = delta;
ci_rate_data(loop_cnt,2,:) = tau_state_goal;
ci_rate_data(loop_cnt,3,:) = msg_drop_prob;
ci_rate_data(loop_cnt,4,:) = mc_sim;

comms_mat = reshape(sum(sum(comms_mat_sent,3),4),[size(comms_mat_sent,1)*size(comms_mat_sent,2),1]);
comms_mat(isnan(comms_mat)) = 0;
msg_data(loop_cnt,5:end) = comms_mat;
comms_rate_mat = reshape(sum(sum(comms_mat_sent,3),4)./sum(sum(comms_mat_total,3),4),[size(comms_mat_sent,1)*size(comms_mat_sent,2),1]);
comms_rate_mat(isnan(comms_rate_mat)) = 0;
msg_rate_data(loop_cnt,5:end) = comms_rate_mat;

ci_data(loop_cnt,5:end) = ci_trigger_vec;
ci_rate_data(loop_cnt,5:end) = ci_trigger_vec ./ length(input_tvec);

loop_cnt = loop_cnt + 1;
last_loop_time = toc;
total_loop_time = total_loop_time + last_loop_time;
avg_loop_time = total_loop_time / loop_cnt;

% save data
save('../../sim_data/fusion_mc_sims_delta2.mat','mse_data','mse_rel_data','baseline_mse_data','baseline_mse_rel_data','msg_data','ci_data','msg_rate_data','ci_rate_data')

end
end
end
end
