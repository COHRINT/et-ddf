% N-agent linear event-triggered cooperative localization
%
% Ian Loefgren
% 12.5.2018
%
% 2D linear cooperative localization problem with N agents

clear; close all; clc;

rng(200);

%% Simulation Parameters

% number of agents in simulation
N = 3;

% connection topology
% chain topology
num_connections = 2;

% time for simulation
max_time = 20;
dt = 0.1;
input_tvec = 0:dt:max_time;


%% Construct Local and Full Systems Models
% the state space model is x and y position and velocity for each agent, x
% and y acceleration input


% local model first
A_local = zeros(4);
A_local(1,2) = 1; A_local(3,4) = 1;

B_local = zeros(4,2);
B_local(2,1) = 1; B_local(4,2) = 1;

Gamma_local = zeros(4,2);
Gamma_local(2,1) = 1; Gamma_local(4,2) = 1;

% each agent gets absolute position updates
C_local = zeros(2,4);
C_local(1,1) = 1; C_local(2,3) = 1;

D_local = 0;

Qct_local = 5;
Rct_local = 1;

[F_local,G_local,H_local,M_local,Q_local,R_local] = ct2dt(A_local,...
                   B_local,C_local,D_local,Gamma_local,Qct_local,Rct_local,dt);
               
% full system model, for all states of all agents
A_full = zeros(4*N);
for i=1:2:4*N
    A_full(i,i+1) = 1;
end
B_full = zeros(4*N,2*N);
Gamma_full = zeros(4*N,2*N);
for i=1:2*N
    B_full(2*i,i) = 1;
    Gamma_full(2*i,i) = 1;
end
% C_full = zeros(4,4*N);
C_full = 1;

D_full = 0;

Qct_full = 5;
Rct_full = eye(1);

[F_full,G_full,H_full,M_full,Q_full,R_full] = ct2dt(A_full,B_full,C_full,D_full,...
                                            Gamma_full,Qct_full,Rct_full,dt);

%% Instantiate filters / agents

agents = cell(N,1);

for i=1:N
    
    % generate starting position
    x_true = [i*10,0,i*10,0]' + mvnrnd([0,0,0,0],5*eye(4))';
    x0_t = mvnrnd(zeros(4*N,1),5*eye(4*N))';
    x0_t((i-1)*4+1:(i-1)*4+4) = x_true; % add a little variation
    
    % create initial uncertainty
    P0_local = 5*eye(4);
    P0_full = 5*eye(4*N);
    
    % simulate inputs
    u((i-1)*2+1:(i-1)*2+2,:) = [2*cos(0.75*input_tvec);2*sin(input_tvec)];
    
    % create agent
    delta = 0;
    agent_id = i;
    % will replace with topology generation fxn
    if i==1
        connections = [-1,2];
    elseif i==N
        connections = [N-1,-1];
    else
        connections = [i-1,i+1];
    end
    
    local_filter = KF(F_local,G_local,H_local,M_local,Q_local,R_local,x_true,P0_local);
    
    common_estimates = {};
    
    cnt = 1;
    for j=1:length(connections)
        if connections(j) > 0
            common_estimates{cnt} = ETKF(F_full,G_full,H_full,M_full,Q_full,R_full,x0_t,P0_full,...
                            delta,agent_id,connections(j));
            cnt = cnt + 1;
        end
    end
    
    
%     common_estimates = {ETKF(F_full,G_full,H_full,M_full,Q_full,R_full,x0_t,P0_full,...
%                         delta,agent_id,connections),...
%                         ETKF(F_full,G_full,H_full,M_full,Q_full,R_full,x0_t,P0_full,...
%                         delta,agent_id,connections)};
                    
    agents{i} = Agent(agent_id,connections,local_filter,common_estimates,x_true);


end


%% Main Simulation Loop

H_rel = [1 0 0 0 -1 0 0 0; 0 0 1 0 0 0 -1 0];

for i = 2:length(input_tvec)
    
    % create measurement inbox
    inbox = cell(N,1);
    
    % process local measurements and determine which to send to connections
    for j=1:length(agents)
        
        % propagate true state
        w = mvnrnd([0,0,0,0],Q_local)';
        agents{j}.true_state(:,end+1) = F_local*agents{j}.true_state(:,end) + G_local*u(2*j-1:2*j,i) + w;
        
        % simulate measurements -> absolute position only for now
        v = mvnrnd([0,0],R_local)';
        y_abs = H_local*agents{j}.true_state(:,end) + v;
        y_abs_msg = struct('src',agents{j}.agent_id,'dest',agents{j}.agent_id,...
                    'status',[],'type',"abs",'data',y_abs);
        msgs = {y_abs_msg};
        
        % relative position
%         rel_msgs = {};
        for k=1:length(agents{j}.connections)
            if agents{j}.connections(k) > 0
                v_rel = mvnrnd([0,0],R_local)';
                y_rel = H_rel*[agents{j}.true_state(:,end); ...
                    agents{agents{j}.connections(k)}.true_state(:,end)] + v_rel;
                y_rel_msg = struct('src',agents{j}.agent_id,'dest',agents{j}.connections(k),...
                    'status',[],'type',"rel",'data',y_rel);
                msgs{end+1} = y_rel_msg;
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
    
    % all agents now process received measurements, performing implicit and
    % explicit measurement updates
    for j=1:length(agents)
        agents{j}.process_received_measurements({inbox{j,:}});
    end
end


% figure
% hold on; grid on;
% for i = 1:length(agents)
%     plot(agents{i}.true_state(1,:),agents{i}.true_state(3,:))
%     plot(agents{i}.local_filter.state_history(1,:),agents{i}.local_filter.state_history(3,:),'.')
% end

figure
hold on; grid on;
plot(agents{2}.true_state(1,:),agents{2}.true_state(3,:))
% plot(agents{1}.common_estimates{1}.state_history(1,:),agents{1}.common_estimates{1}.state_history(3,:))
plot(agents{1}.common_estimates{1}.state_history(5,:),agents{1}.common_estimates{1}.state_history(7,:))

