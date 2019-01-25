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

%% Specify connections

connections = {[3],[3],[1,2,4],[3,5,6],[4],[4]};

% connections = {[2],[1]};

% connections = {[5],[5],[6],[6],[1,2,7],[3,4,7],[5,6,8],[7,9,10],[8,11,12],...
%                 [8,13,14],[9],[9],[10],[10]};

% connections = {[9],[9],[10],[10],[11],[11],[12],[12],...
%                 [1,2,13],[3,4,13],[5,6,14],[7,8,14],[9,10,15],[11,12,15],[13,14,16],...
%                 [15,17,18],[16,19,20],[16,21,22],[17,23,24],[17,25,26],[18,27,28],...
%                 [18,29,30],[19],[19],[20],[20],[21],[21],[22],[22]};

% connections = {[2,4,5],[1,3,6,7],[2,8,9],[1],[1],[2],[2],[3],[3]};
            
% specify which platforms get gps-like measurements
abs_meas_vec = [1 6];

% number of agents
N = length(connections);
% connection topology: tree
num_connections = 3;

% event-triggering params
delta = 2;
tau = 50;
msg_drop_prob = 0;

% simulation params
max_time = 20;
dt = 0.1;
input_tvec = 0:dt:max_time;

%% True starting position and input

for i=1:N
    x_true = [i*10,0.5,i*10,0.5]' + mvnrnd([0,0,0,0],diag([5 1 5 1]))';
    x_true_vec((i-1)*4+1:(i-1)*4+4,1) = x_true;
    
    % generate input for platforms
    u((i-1)*2+1:(i-1)*2+2,:) = [2*cos(0.75*input_tvec);2*sin(input_tvec)];
end

%% Create centralized KF

R_abs = 1*eye(2);
R_rel = 3*eye(2);

% generate dynamics matrices for baseline filter
[F_full,G_full] = ncv_dyn(dt,N);
Q_full = 1*eye(4*N);
x0_full = x_true_vec;
P0_full = 100*eye(4*N);

% create baseline filter for comparison
baseline_filter = KF(F_full,G_full,0,0,Q_full,R_abs,R_rel,x_true_vec,P0_full,0);


%% Create agents objects

% for each platform, create dynamics models, and filters
Q_local = [0.0003 0.005 0 0;
            0.005 0.1 0 0;
            0 0 0.0003 0.005;
            0 0 0.005 0.1];

agents = cell(N,1);
for i=1:N
    
    agent_id = i;
    
    % construct local estimates
    n = (length(connections{i})+1)*4;
    [F,G] = ncv_dyn(dt,length(connections{i})+1);
    Q_localfilter = 2*eye(n);
%     Q_localfilter_cell = cell(1,length(connections{i})+1);
%     [Q_localfilter_cell{:}] = deal(Q_local);
%     Q_localfilter = blkdiag(Q_localfilter_cell{:});
    
    x0 = [x_true_vec((agent_id-1)*4+1:(agent_id-1)*4+4,1)];
    for j=1:length(connections{i})
        if connections{i}(j) > agent_id
            x0 = [x0; x_true_vec((connections{i}(j)-1)*4+1:(connections{i}(j)-1)*4+4,1)];
        else
            x0 = [x_true_vec((connections{i}(j)-1)*4+1:(connections{i}(j)-1)*4+4,1); x0];
        end
    end
    P0 = 100*eye(4*(length(connections{i})+1));
    
    local_filter = ETKF(F,G,0,0,Q_localfilter,R_abs,R_rel,x0,P0,delta,agent_id,0);
    
    % construct common estimates, will always only have two agents
    [F_comm,G_comm] = ncv_dyn(dt,2);
    Q_comm = 2*eye(8);
%     Q_comm = blkdiag(Q_local,Q_local);
    common_estimates = {};
    for j=1:length(connections{i})
        % make sure common estimate state vector is ordered by agent id
        if connections{i}(j) > agent_id
            x0_comm = [x_true_vec((agent_id-1)*4+1:(agent_id-1)*4+4,1); x_true_vec((connections{i}(j)-1)*4+1:(connections{i}(j)-1)*4+4,1)];
        else
            x0_comm = [x_true_vec((connections{i}(j)-1)*4+1:(connections{i}(j)-1)*4+4,1); x_true_vec((agent_id-1)*4+1:(agent_id-1)*4+4,1)];
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
        w = mvnrnd([0,0,0,0],Q_local)';
        agents{j}.true_state(:,end+1) = F_local*agents{j}.true_state(:,end) + G_local*u(2*(j-1)+1:2*(j-1)+2,i) + w;
        
        % simulate measurements: absolute measurements
%         if ((j == 1) ||  (j == 2))
        if ismember(j,abs_meas_vec)
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

        %% Process the generated measurements locally, determine which to send
        input = u(2*(j-1)+1:2*(j-1)+2,i);
%         input = zeros(2,1);
        outgoing = agents{j}.process_local_measurements(input,msgs);
        
%         add outgoing measurements to each agents "inbox"
        for k=1:length(outgoing)
            dest = outgoing{k}.dest;
            inbox{dest,j} = outgoing{k};
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
        if trace(agents{j}.local_filter.P*diag(alpha)) > tau
            agents{j}.ci_trigger_cnt = agents{j}.ci_trigger_cnt + 1;
            ci_trigger_list(j) = 1;
            
            % determine common states and grab snapshot of those common
            % states
            % save snapshot of state estimate and covariance, send to
            % connection inboxes
            x_snap = agents{j}.local_filter.x;
            P_snap = agents{j}.local_filter.P;
            for k=1:length(agents{j}.connections)
                
                % compute transforms for platform and connection
                Ta = gen_sim_transform(agents{j}.agent_id,agents{j}.connections,...
                    agents{agents{j}.connections(k)}.agent_id,agents{agents{j}.connections(k)}.connections);
                Tb = gen_sim_transform(agents{agents{j}.connections(k)}.agent_id,...
                    agents{agents{j}.connections(k)}.connections,agents{j}.agent_id,agents{j}.connections);
                
                % compute intersection of states (includes connections and
                % agent ids for both platforms)
                inter = intersect(sort([agents{j}.connections,agents{j}.agent_id]),...
                        sort([agents{agents{j}.connections(k)}.connections,agents{agents{j}.connections(k)}.agent_id]));
                
                % transform means and covariances to group common states at
                % beginning of state vector/covariance
                xaT = Ta\x_snap;
                xaTred = xaT(1:8);
                PaT = Ta\P_snap*Ta;
                PaTred = PaT(1:8,1:8);
                
%                 ci_inbox{agents{j}.connections(k)}{end+1} = {x_snap,P_snap,agents{j}.agent_id,agents{j}.connections};
                ci_inbox{agents{j}.connections(k)}{end+1} = {xaTred,PaTred,agents{j}.agent_id,agents{j}.connections};
                
                x_conn_snap = agents{agents{j}.connections(k)}.local_filter.x;
                P_conn_snap = agents{agents{j}.connections(k)}.local_filter.P;
                
                xbT = Tb\x_conn_snap;
                xbTred = xbT(1:8);
                PbT = Tb\P_conn_snap*Tb;
                PbTred = PbT(1:8,1:8);
                
%                 ci_inbox{j}{end+1} = {x_conn_snap,P_conn_snap,agents{j}.connections(k),agents{agents{j}.connections(k)}.connections};
                ci_inbox{j}{end+1} = {xbTred,PbTred,agents{j}.connections(k),agents{agents{j}.connections(k)}.connections};
                
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
                
                % construct transformation matrices
%                 agent_loc = find(sort([obj.connections,obj.agent_id]) == obj.agent_id);

                % a
%                 a_conn = sort([agents{j}.connections,agents{j}.agent_id]); % construct ordered list of all agent ids, including self and connections
%                 
%                 a_loc = find(a_conn == agents{j}.agent_id); % find location in ordered list of self
%                 b_loc = find(a_conn == b_id); % find location of target to fuse with
%                 
%                 a_conn_loc = 1:length(a_conn);
%                 a_conn_loc(a_conn_loc == find(a_conn == agents{j}.agent_id)) = []; % remove self agent id from list
%                 a_conn_loc(a_conn_loc == find(a_conn == b_id)) = []; % remove fuse target id from list
%                 
%                 if a_loc < b_loc
%                     a_conn_loc = [a_loc,b_loc,a_conn_loc]; % add removed ids back, at beginning of list
%                 elseif b_loc < a_loc
%                     a_conn_loc = [b_loc,a_loc,a_conn_loc];
%                 end
%                 
%                 Ta = zeros(size(Pa));
%                 for ii=1:length(a_conn_loc)
%                     Ta(4*(a_conn_loc(ii)-1)+1:4*(a_conn_loc(ii)-1)+4,4*(ii-1)+1:4*(ii-1)+4) = eye(4);
%                 end
                Ta = gen_sim_transform(agents{j}.agent_id,agents{j}.connections,b_id,b_connections);
                
                % b 
%                 b_conn = sort([b_connections,b_id]); % construct ordered list of all agent ids, including self and connections
%                 
%                 a_loc = find(b_conn == agents{j}.agent_id); % find location in ordered list of self
%                 b_loc = find(b_conn == b_id); % find location of target to fuse with
%                 
%                 b_conn_loc = 1:length(b_conn);
%                 b_conn_loc(b_conn_loc == find(b_conn == agents{j}.agent_id)) = []; % remove self agent id from list
%                 b_conn_loc(b_conn_loc == find(b_conn == b_id)) = []; % remove fuse target id from list
%                 
%                 if a_loc < b_loc
%                     b_conn_loc = [a_loc,b_loc,b_conn_loc]; % add removed ids back, at beginning of list
%                 elseif b_loc < a_loc
%                     b_conn_loc = [b_loc,a_loc,b_conn_loc];
%                 end
%                 
%                 Tb = zeros(size(Pb));
%                 for ii=1:length(b_conn_loc)
%                     Tb(4*(b_conn_loc(ii)-1)+1:4*(b_conn_loc(ii)-1)+4,4*(ii-1)+1:4*(ii-1)+4) = eye(4);
%                 end
%                 
%                 if (size(Ta,1) ~= size(xa,1)) || (size(Tb,1) ~= size(xb,1))
%                     disp('break!')
%                 end
%                 Tb = gen_sim_transform(b_id,b_connections,agents{j}.agent_id,agents{j}.connections);
                
                % transform means and covariances to group common states at
                % beginning of state vector/covariance
                xaT = Ta\xa;
                xaTred = xaT(1:8);
                PaT = Ta\Pa*Ta;
                PaTred = PaT(1:8,1:8);
                
%                 xbT = Tb\xb;
%                 xbTred = xbT(1:8);
%                 PbT = Tb\Pb*Tb;
%                 PbTred = PbT(1:8,1:8);
                
                xbTred = xb;
                PbTred = Pb;

                alpha = ones(size(PaTred,1),1);
%                 [xc,Pc] = covar_intersect(xaTred,xbTred,PaTred,PbTred,alpha);
                [xc,Pc] = covar_intersect(xaTred,xbTred,PaTred,PbTred,alpha);
                
                % compute information delta for conditional update
                invD = inv(Pc)-inv(PaTred);
                invDd = inv(Pc)*xc - inv(PaTred)*xaTred;
                
                % conditional gaussian update
%                 V = inv(inv(PaT) + [inv(Pc) zeros(size(Pc,1),size(PaT,2)-size(Pc,2)); ...
%                     zeros(size(PaT,1)-size(Pc,1),size(Pc,2)) zeros(size(PaT)-size(Pc))]);
%                 v = V*(PaT\xaT + [Pc\xc; zeros(size(PaT,1)-size(Pc,1),1)]);

                V = inv(inv(PaT) + [invD zeros(size(Pc,1),size(PaT,2)-size(Pc,2)); ...
                    zeros(size(PaT,1)-size(Pc,1),size(Pc,2)) zeros(size(PaT)-size(Pc))]);
                v = V*(PaT\xaT + [invDd; zeros(size(PaT,1)-size(Pc,1),1)]);
                
                
                % transform back to normal state order
                xa = Ta*v;
                Pa = Ta*V/Ta;
                
%                 xb = Tb*v;
%                 Pb = Tb*V/Tb;
                
                % update local estimates
                agents{j}.local_filter.x = xa;
                agents{j}.local_filter.P = Pa;
                
                %                   agents{agents{j}.connections(k)}.local_filter.x = xc;
                %                   agents{agents{j}.connections(k)}.local_filter.P = Pc;
                
                % update common estimates
                for ii=1:length(agents{j}.common_estimates)
                    if agents{j}.common_estimates{ii}.connection == ci_inbox{j}{k}{3}
                        agents{j}.common_estimates{ii}.x = xc;
                        agents{j}.common_estimates{ii}.P = Pc;
                    end
                end
            end
        end
    end

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

