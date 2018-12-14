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
N = 30;
% connection topology
% chain topology
num_connections = 2;
delta = 3;
tau = 20000;
msg_drop_prob = 0;

% time for simulationo
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

Qct_local = 1;
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

% connections = {[9],[9],[10],[10],[11],[11],[12],[12],...
%                 [1,2,13],[3,4,13],[5,6,14],[7,8,14],[9,10,15],[11,12,15],[13,14,16],...
%                 [15,17,18],[16,19,20],[16,21,22],[17,23,24],[17,25,26],[18,27,28],...
%                 [18,29,30],[19],[19],[20],[20],[21],[21],[22],[22]};

% connections = {[3],[3],[1,2,4],[3,5,6],[4],[4]};

% connections = {[5],[5],[6],[6],[1,2,7],[3,4,7],[5,6,8],[7,9,10],[8,11,12],...
%                 [8,13,14],[9],[9],[10],[10]};

for i=1:N
    x_true = [i*10,0,i*10,0]' + mvnrnd([0,0,0,0],5*eye(4))';
    x_true_vec((i-1)*4+1:(i-1)*4+4,1) = x_true;
end

% R_abs = (1.25^2)*eye(2);
% R_rel = (0.75^2)*eye(2);

R_abs = 9*eye(2);
R_rel = 25*eye(2);

for i=1:N
    
    % generate starting position
%     x_true = [i*10,0,i*10,0]' + mvnrnd([0,0,0,0],5*eye(4))';
%     x0_t = mvnrnd(zeros(4*N,1),5*eye(4*N))';
%     x_true((i-1)*4+1:(i-1)*4+4); % add a little variation
    
    % create initial uncertainty
    P0_local = 500*eye(4);
    P0_full = 100*eye(4*N);
    
    % simulate inputs
    u((i-1)*2+1:(i-1)*2+2,:) = [2*cos(0.75*input_tvec);2*sin(input_tvec)];
    
    baseline_filter = KF(F_full,G_full,H_full,M_full,Q_full,R_abs,R_rel,x_true_vec,P0_full,0);
    
    % create agent
%     delta = 1;
    agent_id = i;
    % will replace with topology generation fxn
    if i==1
        connections = [2];
    elseif i==N
        connections = [N-1];
    else
        connections = [i-1,i+1];
    end
    
%     local_filter = KF(F_local,G_local,H_local,M_local,Q_local,R_local,x_true_vec((i-1)*4+1:(i-1)*4+4)',P0_local);
    local_filter = ETKF(F_full,G_full,H_full,M_full,Q_full,R_abs,R_rel,x_true_vec,P0_full,delta,agent_id,0);
    
    common_estimates = {};
    
    cnt = 1;
    for j=1:length(connections)
        if connections(j) > 0
            common_estimates{cnt} = ETKF(F_full,G_full,H_full,M_full,Q_full,R_abs,R_rel,x_true_vec,P0_full,...
                            delta,agent_id,connections(j));
            cnt = cnt + 1;
        end
    end
    
    
%     common_estimates = {ETKF(F_full,G_full,H_full,M_full,Q_full,R_full,x0_t,P0_full,...
%                         delta,agent_id,connections),...
%                         ETKF(F_full,G_full,H_full,M_full,Q_full,R_full,x0_t,P0_full,...
%                         delta,agent_id,connections)};
                    
    agents{i} = Agent(agent_id,connections,local_filter,common_estimates,x_true_vec((i-1)*4+1:(i-1)*4+4,1),msg_drop_prob);


end


%% Main Simulation Loop

H_rel = [1 0 0 0 -1 0 0 0; 0 0 1 0 0 0 -1 0];

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
        
        % simulate measurements -> absolute position only for now
        if ((j == 11) ||  (j == 18))
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
        alpha = ones(4*N,1);
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

%% save results to mat file

filename = strcat('../test_data/N',num2str(N),'_d',num2str(delta),'_t',num2str(tau),'_',datestr(now,'yy:mm:dd:HH:MM:SS'),'.mat');
% save(filename);



