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
load('sensor_noise_data.mat');

rng(200)

%% Specify connections

% connections = {[3],[3],[1,2,4],[3,5,6],[4],[4]};

% connections = {[2],[1,3],[2,4],[3]};

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
num_connections = 3;

% event-triggering params
delta = 3;
tau_goal = 100;
tau = 100;
msg_drop_prob = 0;

% simulation params
max_time = 20;
dt = 0.1;
input_tvec = 0:dt:max_time;

%% True starting position and input

for i=1:N
    x_true = [i*10,0,i*10,0]' + mvnrnd([0,0,0,0],diag([5 5 5 5]))';
    x_true_vec((i-1)*4+1:(i-1)*4+4,1) = x_true;
    
    % generate input for platforms
    u((i-1)*2+1:(i-1)*2+2,:) = [2*cos(0.75*input_tvec);2*sin(input_tvec)];
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

R_abs = 1*eye(2);
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
    
    % construct local estimates
    n = (length(connections{i})+1)*4;
    [F,G] = ncv_dyn(dt,length(connections{i})+1);
%     Q_localfilter = 1*eye(n);
    Q_localfilter_cell = cell(1,length(connections{i})+1);
    [Q_localfilter_cell{:}] = deal(Q_local);
    Q_localfilter = blkdiag(Q_localfilter_cell{:});

    ids = sort([agent_id,connections{i}]);
    x0 = [];
    for j=1:length(ids)
        x0 = [x0; x_true_vec((ids(j)-1)*4+1:(ids(j)-1)*4+4,1)];
    end
    
    P0 = 100*eye(4*(length(connections{i})+1));
    
    local_filter = ETKF(F,G,0,0,Q_localfilter,R_abs,R_rel,x0,P0,delta,agent_id,connections{i});
    
    % construct common estimates, will always only have two agents
    [F_comm,G_comm] = ncv_dyn(dt,2);
%     Q_comm = 1*eye(8);
    Q_comm = blkdiag(Q_local,Q_local);
    common_estimates = {};
    
    for j=1:length(connections{i})
        
        % make sure common estimate state vector is ordered by agent id
        comm_ids = sort([agent_id,connections{i}(j)]);
        x0_comm = [];
        
        for k=1:length(comm_ids)
            x0_comm = [x0_comm; x_true_vec((comm_ids(k)-1)*4+1:(comm_ids(k)-1)*4+4,1)];
        end

        P0_comm = 100*eye(8);
        common_estimates{j} = ETKF(F_comm,G_comm,0,0,Q_comm,R_abs,R_rel,x0_comm,P0_comm,delta,agent_id,connections{i}(j));
    end
    
    agents{i} = Agent(agent_id,connections{i},local_filter,common_estimates,x_true_vec((i-1)*4+1:(i-1)*4+4,1),msg_drop_prob,tau_goal,tau);
    
end

%% Main Simulation Loop

H_local = [1 0 0 0; 0 0 1 0];
H_rel = [1 0 0 0 -1 0 0 0; 0 0 1 0 0 0 -1 0];
[F_local,G_local] = ncv_dyn(dt);
% Q_local = 0.1*eye(4);


ci_time_vec = zeros(N,length(input_tvec));

for i = 2:length(input_tvec)
%     tic
    clc;
    fprintf('Time step %i of %i, %f seconds of %f total\n',i,length(input_tvec),i*dt,length(input_tvec)*dt);
    
    % create measurement inbox
    inbox = cell(N,1);
    
    baseline_filter.predict(u(:,i));
%     baseline_filter.predict(zeros(size(u(:,i))));
    
    % process local measurements and determine which to send to connections
    for j=1:length(agents)
        msgs = {};
        
        % propagate true state
        w = mvnrnd([0,0,0,0],Q_local_true)';
%         w = w_data{j}(:,i);
        agents{j}.true_state(:,end+1) = F_local*agents{j}.true_state(:,end) + G_local*u(2*(j-1)+1:2*(j-1)+2,i) + w;
        
        % simulate measurements: absolute measurements
        if ismember(agents{j}.agent_id,abs_meas_vec)
            v = mvnrnd([0,0],R_abs)';
%             v = v_data{j}(:,i);
            y_abs = H_local*agents{j}.true_state(:,end) + v;
            y_abs_msg = struct('src',agents{j}.agent_id,'dest',agents{j}.agent_id,...
                        'status',[1 1],'type',"abs",'data',y_abs);
            msgs = {y_abs_msg};
            
            baseline_filter.update(y_abs,'abs',agents{j}.agent_id,agents{j}.agent_id);
        end
        
        % relative position
        for k=1:length(agents{j}.connections)
            if agents{j}.connections(k) > 0
                v_rel = mvnrnd([0,0],R_rel)';
%                 v_rel = v_rel_data{j}(:,i);
                y_rel = H_rel*[agents{j}.true_state(:,end); ...
                    agents{agents{j}.connections(k)}.true_state(:,end)] + v_rel;
                y_rel_msg = struct('src',agents{j}.agent_id,'dest',agents{j}.connections(k),...
                    'status',[1 1],'type',"rel",'data',y_rel);
                msgs{end+1} = y_rel_msg;
                
                baseline_filter.update(y_rel,'rel',agents{j}.agent_id,agents{j}.connections(k));
            end
        end

        %% Process the generated measurements locally, determine which to send
        input = u(2*(j-1)+1:2*(j-1)+2,i);
%         input = zeros(2,1);
        outgoing = agents{j}.process_local_measurements(input,msgs);
        
%         add outgoing measurements to each agents "inbox"
        for k=1:length(outgoing)
            dest = outgoing{k}.dest;
            inbox{dest,end+1} = outgoing{k};
        end
    end
    
    %% All agents now process received measurements, performing implicit and
    % explicit measurement updates
    for j=1:length(agents)
        agents{j}.process_received_measurements({inbox{j,:}});
    end
    
    %% Covariance intersection thresholding and snapshotting
    ci_inbox = cell(N,1);
    inbox_ind = 1;
    % covariance intersection between agents
    ci_trigger_list = zeros(1,length(agents));
    
    for j=1:length(agents)
        alpha = ones(4*(length(agents{j}.connections)+1),1);

        % check trace of cov to determine if CI should be triggered
        if trace(agents{j}.local_filter.P*diag(alpha)) > agents{j}.tau
            agents{j}.ci_trigger_cnt = agents{j}.ci_trigger_cnt + 1;
            ci_trigger_list(j) = 1;
            ci_trigger_mat(j,i) = 1;
            
            % determine common states and grab snapshot of those common
            % states
            % save snapshot of state estimate and covariance, send to
            % connection inboxes
            x_snap = agents{j}.local_filter.x;
            P_snap = agents{j}.local_filter.P;
            for k=1:length(agents{j}.connections)
    
                % compute transforms for platform and connection, and
                % number of intersecting states
                
                [Ta,il_a] = gen_sim_transform(agents{j}.agent_id,agents{j}.connections,...
                    agents{agents{j}.connections(k)}.agent_id,agents{agents{j}.connections(k)}.connections);
                [Tb,il_b] = gen_sim_transform(agents{agents{j}.connections(k)}.agent_id,...
                    agents{agents{j}.connections(k)}.connections,agents{j}.agent_id,agents{j}.connections);

                % transform means and covariances to group common states at
                % beginning of state vector/covariance
                xaT = Ta\x_snap;
                xaTred = xaT(1:il_a);
                PaT = Ta\P_snap*Ta;
                PaTred = PaT(1:il_a,1:il_a);
                
%                 ci_inbox{agents{j}.connections(k)}{end+1} = {x_snap,P_snap,agents{j}.agent_id,agents{j}.connections};
                ci_inbox{agents{j}.connections(k)}{end+1} = {xaTred,PaTred,agents{j}.agent_id,agents{j}.connections,agents{j}.tau};
                
                x_conn_snap = agents{agents{j}.connections(k)}.local_filter.x;
                P_conn_snap = agents{agents{j}.connections(k)}.local_filter.P;
                
                xbT = Tb\x_conn_snap;
                xbTred = xbT(1:il_b);
                PbT = Tb\P_conn_snap*Tb;
                PbTred = PbT(1:il_b,1:il_b);
                
%                 ci_inbox{j}{end+1} = {x_conn_snap,P_conn_snap,agents{j}.connections(k),agents{agents{j}.connections(k)}.connections};
                ci_inbox{j}{end+1} = {xbTred,PbTred,agents{j}.connections(k),agents{agents{j}.connections(k)}.connections,agents{agents{j}.connections(k)}.tau};
                
                if isempty(agents{agents{j}.connections(k)}.connections)
                    disp('break')
                end
                
%                 disp(ci_inbox{j}{end}{4})
                
            end
        end
    end
    
    %% Acutal covariance intersection performed (w/ conditional updates on full states)
    for j=1:length(agents)
        for k=1:length(ci_inbox{j})
            if ~isempty(ci_inbox{j}{k})
                xa = agents{j}.local_filter.x;
                Pa = agents{j}.local_filter.P;
                
                xb = ci_inbox{j}{k}{1};
                Pb = ci_inbox{j}{k}{2};
                b_id = ci_inbox{j}{k}{3};
                b_connections = ci_inbox{j}{k}{4};
                b_tau = ci_inbox{j}{k}{5};
                
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
                
                % update local estimates
                agents{j}.local_filter.x = xa;
                agents{j}.local_filter.P = Pa;
                
                % update common estimates
                for ii=1:length(agents{j}.common_estimates)
                    if agents{j}.common_estimates{ii}.connection == ci_inbox{j}{k}{3}
%                         conn_loc = inter==agents{j}.common_estimates{ii}.connection;
                        agents{j}.common_estimates{ii}.x = xc;
                        agents{j}.common_estimates{ii}.P = Pc;
                        
                        agents{j}.connection_taus(ii) = b_tau;
                    end
                end
                
                % update CI threshold tau
%                 agents{j}.tau = min(agents{j}.tau_goal,agents{j}.tau + ...
%                     agents{j}.epsilon_1*sum(agents{j}.connection_taus-agents{j}.tau*ones(length(agents{j}.connection_taus),1)) + ...
%                     agents{j}.epsilon_2*(agents{j}.tau_goal-agents{j}.tau));
                
            end
        end
%         agents{j}.tau = min(agents{j}.tau_goal,agents{j}.tau + ...
%                     agents{j}.epsilon_1*sum(-agents{j}.connection_taus+agents{j}.tau*ones(length(agents{j}.connection_taus),1)) + ...
%                     agents{j}.epsilon_2*(agents{j}.tau_goal-agents{j}.tau));
    end

    for j=1:length(agents)
        agents{j}.local_filter.state_history(:,i) = agents{j}.local_filter.x;
        agents{j}.local_filter.cov_history(:,:,i) = agents{j}.local_filter.P;
        agents{j}.tau_history(i) = agents{j}.tau;
        
        for k=1:length(agents{j}.common_estimates)
            agents{j}.common_estimates{k}.state_history(:,i) = agents{j}.common_estimates{k}.x;
            agents{j}.common_estimates{k}.cov_history(:,:,i) = agents{j}.common_estimates{k}.P;
        end
    end
              
%     toc
end

