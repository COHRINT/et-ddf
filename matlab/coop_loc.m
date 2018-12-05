% 2D cooperative localization
%
% Ian Loefgren
% 11.27.2018
%
% 2D cooperative localization problem with 2 agents

clear; close all; clc;

rng(200);

% CT model - shared by both agents
% state is: [xpos xdot ypos ydot xpos_target xdot_target ypos_target ydot_target]^T
A = zeros(8);
A(1,2) = 1; A(3,4) = 1; A(5,6) = 1; A(7,8) = 1;

B = zeros(8,4);
B(2,1) = 1; B(4,2) = 1; B(6,3) = 1; B(8,4) = 1;

Gamma = zeros(8,4);
Gamma(2,1) = 1; Gamma(4,2) = 1; Gamma(6,3) = 1; Gamma(8,4) = 1;

C = zeros(4,8);
C(1,1) = 1; C(2,3) = 1; C(3,5) = 1; C(4,7) = 1;

D = 0;

Qct = 5;
Rct = 1;

dt = 0.1;

[F,G,H,M,Q,R] = ct2dt(A,B,C,D,Gamma,Qct,Rct,dt);

% generate input
input_tvec = 0:dt:20;
u1 = 2*cos(0.75*input_tvec);
u2 = 2*sin(input_tvec);
u3 = 2*cos(.5*input_tvec);
u4 = 2*sin(input_tvec);
u = [u1;u2;u3;u4];

%% Simulation
x_t1 = mvnrnd([0,0,0,0],eye(4))';
x_t2 = mvnrnd([30,0,30,0],eye(4))';

for i=1:length(input_tvec)
    
    % simulate process noise
    % agent 1
    w1 = mvnrnd(zeros(1,4),Q(1:4,1:4))';
    x_true1 = F(1:4,1:4)*x_t1 + G(1:4,1:2)*u(1:2,i) + w1;
    
    % agent 2
    w2 = mvnrnd(zeros(1,4),Q(5:8,5:8))';
    x_true2 = F(5:8,5:8)*x_t2 + G(5:8,3:4)*u(3:4,i) + w2;
    
    % simulate measurement noise
    v1 = mvnrnd(zeros(1,2),R(1:2,1:2))';
    y_meas_obs1 = H(1:2,1:4)*x_true1 + v1;
    v2 = mvnrnd(zeros(1,2),R(3:4,3:4))';
    y_meas_obs2 = H(3:4,5:8)*x_true2 + v2;
    
    x_true1_vec(:,i) = x_true1;
    x_true2_vec(:,i) = x_true2;
    y_meas_vec_1(:,i) = y_meas_obs1;
    y_meas_vec_2(:,i) = y_meas_obs2;
    x_t1 = x_true1;
    x_t2 = x_true2;
    
end

%% Instantiate filters

% baseline kf
x0 = [zeros(4,1);[30,0,30,0]'];
P0 = eye(8);
baseline = KF(F,G,H,M,Q,R,x0,P0);

% ETKF agent 1
delta = 0;
x01 = [zeros(4,1);[30,0,30,0]'];
P01 = eye(8);
agent1 = ETKF(F,G,H,M,Q,R,x01,P01,delta);

% agent 1 common information
delta = 6.25;
x01 = [zeros(4,1);[30,0,30,0]'];
P01 = eye(8);
agent1_common = ETKF(F,G,H,M,Q,R,x01,P01,delta);

% ETKF agent 2
delta = 0;
x02 = [zeros(4,1);[30,0,30,0]'];
P02 = eye(8);
agent2 = ETKF(F,G,H,M,Q,R,x02,P02,delta);

% agent 2 common information
delta = 6.25;
tau = 100;
x02 = [zeros(4,1);[30,0,30,0]'];
P02 = eye(8);
agent2_common = ETKF(F,G,H,M,Q,R,x02,P02,delta);

x_base(:,1) = x0; P_base(:,:,1) = P0;
x_est_1(:,1) = x01; P_est_1(:,:,1) = P01;
x_est_2(:,1) = x02; P_est_2(:,:,1) = P02;
x_est_1_common(:,1) = x01; P_est_1_common(:,:,1) = P01;
x_est_2_common(:,1) = x02; P_est_2_common(:,:,1) = P02;

ci_trigger_cnt = 0;
total_cnt = 0;

for i = 2:length(input_tvec)
    
    % full kf baseline
    baseline.predict(u(:,i));
    [x_curr,P_curr] = baseline.update([y_meas_vec_1(:,i);y_meas_vec_2(:,i)]);
    x_base(:,i) = x_curr; P_base(:,:,i) = P_curr;
    
    % agent 1 etkf
    agent1.predict([u(1:2,i);0;0]);
    [x_curr,P_curr] = agent1.update(y_meas_vec_1(:,i),1);
    x_est_1(:,i) = x_curr; P_est_1(:,:,i) = P_curr;
    
    % agent 1 common information -> potentially sent measurements from
    % agent 2
    agent1_common.predict([u(1:2,i);0;0]);
    [x_curr,P_curr] = agent1_common.update(y_meas_vec_2(:,i),3);
    x_est_1_common(:,i) = x_curr; P_est_1_common(:,:,i) = P_curr;
    
    % agent 2 etkf
    agent2.predict([0;0;u(3:4,i)]);
    [x_curr,P_curr] = agent2.update(y_meas_vec_2(:,i),3);
    x_est_2(:,i) = x_curr; P_est_2(:,:,i) = P_curr;
    
    % agent 1 common information -> potentially sent measurements from
    % agent 2
    agent2_common.predict([0;0;u(3:4,i)]);
    [x_curr,P_curr] = agent2_common.update(y_meas_vec_1(:,i),1);
    x_est_2_common(:,i) = x_curr; P_est_2_common(:,:,i) = P_curr;
    
    if trace(agent1_common.P) > tau || trace(agent2_common.P) > tau
        
        [xc,Pc] = covar_intersect(agent1_common.x,agent2_common.x,agent1_common.P,agent2_common.P);
        agent1_common.x = xc; agent2_common.x = xc;
        agent1_common.P = Pc; agent2_common.P = Pc;
        
        x_est_1_common(:,i) = xc; P_est_1_common(:,:,i) = Pc;
        x_est_2_common(:,i) = xc; P_est_2_common(:,:,i) = Pc;
        
        ci_trigger_cnt = ci_trigger_cnt + 1;
    end
    total_cnt = total_cnt + 1;
        
end

fprintf('total ci triggers, %i\n',ci_trigger_cnt)
fprintf('msgs sent to agent 1 by type: %i %i %i %i %i %i %i %i\n',...
        agent1_common.msg_sent(1),agent1_common.msg_sent(2),agent1_common.msg_sent(3),...
        agent1_common.msg_sent(4),agent1_common.msg_sent(5),agent1_common.msg_sent(6),...
        agent1_common.msg_sent(7),agent1_common.msg_sent(8))
fprintf('msgs sent to agent 2 by type: %i %i %i %i %i %i %i %i\n',...
        agent2_common.msg_sent(1),agent2_common.msg_sent(2),agent2_common.msg_sent(3),...
        agent2_common.msg_sent(4),agent2_common.msg_sent(5),agent2_common.msg_sent(6),...
        agent2_common.msg_sent(7),agent2_common.msg_sent(8))

figure
hold on; grid on;
plot(x_true1_vec(1,:),x_true1_vec(3,:))
plot(x_true2_vec(1,:),x_true2_vec(3,:))
legend('Agent 1','Agent 2')
xlabel('x [m]')
ylabel('y [m]')
title('Plaftorm trajectories')

figure

%% Plots

% agent 1 plots

% x pos
subplot(4,1,1)
hold on; grid on;
plot(input_tvec,x_est_1(1,:) - x_true1_vec(1,:))
plot_xpos_cov_1(:) = sqrt(P_est_1(1,1,:));
plot(input_tvec,2*plot_xpos_cov_1,'r--')
plot(input_tvec,-2*plot_xpos_cov_1,'r--')
fill([input_tvec flip(input_tvec)],[2*plot_xpos_cov_1 -2*plot_xpos_cov_1],'r','LineStyle','none')
alpha(0.25)
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_base(1,:) - x_true1_vec(1,:),'-.m')
% plot(input_tvec,x_est_2(1,:) - x_true1_vec(1,:))
plot(input_tvec,x_est_2_common(1,:) - x_true1_vec(1,:))
plot(input_tvec,2*sqrt(squeeze(P_est_2_common(1,1,:))'),'g--')
plot(input_tvec,-2*sqrt(squeeze(P_est_2_common(1,1,:))'),'g--')
fill([input_tvec flip(input_tvec)],[2*sqrt(squeeze(P_est_2_common(1,1,:))')-2*plot_xpos_cov_1+2*plot_xpos_cov_1 zeros(size(-2*sqrt(squeeze(P_est_2_common(1,1,:))')))],'g','LineStyle','none')
alpha(0.25)
fill([input_tvec flip(input_tvec)],[-2*sqrt(squeeze(P_est_2_common(1,1,:))')+2*plot_xpos_cov_1 zeros(size(-2*sqrt(squeeze(P_est_2_common(1,1,:))')))],'g','LineStyle','none')
alpha(0.25)
xlabel('Time [s]')
ylabel('Pos error [m]')
title(['Agent1 est X position error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(sum(agent1_common.msg_sent)),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('Agent1 est','\pm 2\sigma','','truth','KF est','Agent2 common est','common \pm 2\sigma')

% x velocity
subplot(4,1,2)
hold on; grid on;
plot(input_tvec,x_est_1(2,:) - x_true1_vec(2,:))
plot_xvel_cov_1(:) = sqrt(P_est_1(2,2,:));
plot(input_tvec,2*plot_xvel_cov_1,'r--')
plot(input_tvec,-2*plot_xvel_cov_1,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_base(2,:) - x_true1_vec(2,:),'-.m')
% plot(input_tvec,x_est_2(2,:) - x_true1_vec(2,:))
plot(input_tvec,x_est_2_common(2,:) - x_true1_vec(2,:))
plot(input_tvec,2*sqrt(squeeze(P_est_2_common(2,2,:))'),'g--')
plot(input_tvec,-2*sqrt(squeeze(P_est_2_common(2,2,:))'),'g--')
xlabel('Time [s]')
ylabel('Vel error [m/s]')
title(['Agent1 est X velocity error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(sum(agent1_common.msg_sent)),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('Agent1 est','\pm 2\sigma','','truth','KF est','Agent2 common est','common \pm 2\sigma')

subplot(4,1,3)
hold on; grid on;
plot(input_tvec,x_est_1(3,:) - x_true1_vec(3,:))
plot_ypos_cov_1(:) = sqrt(P_est_1(3,3,:));
plot(input_tvec,2*plot_ypos_cov_1,'r--')
plot(input_tvec,-2*plot_ypos_cov_1,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_base(3,:) - x_true1_vec(3,:),'-.m')
% plot(input_tvec,x_est_2(3,:) - x_true1_vec(3,:))
plot(input_tvec,x_est_2_common(3,:) - x_true1_vec(3,:))
plot(input_tvec,2*sqrt(squeeze(P_est_2_common(3,3,:))'),'g--')
plot(input_tvec,-2*sqrt(squeeze(P_est_2_common(3,3,:))'),'g--')
xlabel('Time [s]')
ylabel('Pos error [m]')
title(['Agent1 est Y position error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(sum(agent1_common.msg_sent)),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('Agent1 est','\pm 2\sigma','','truth','KF est','Agent2 common est','common \pm 2\sigma')

subplot(4,1,4)
hold on; grid on;
plot(input_tvec,x_est_1(4,:) - x_true1_vec(4,:))
plot_yvel_cov_1(:) = sqrt(P_est_1(4,4,:));
plot(input_tvec,2*plot_yvel_cov_1,'r--')
plot(input_tvec,-2*plot_yvel_cov_1,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_base(4,:) - x_true1_vec(4,:),'-.m')
% plot(input_tvec,x_est_2(4,:) - x_true1_vec(4,:))
plot(input_tvec,x_est_2_common(4,:) - x_true1_vec(4,:))
plot(input_tvec,2*sqrt(squeeze(P_est_2_common(4,4,:))'),'g--')
plot(input_tvec,-2*sqrt(squeeze(P_est_2_common(4,4,:))'),'g--')
xlabel('Time [s]')
ylabel('Vel error [m]')
title(['Agent1 est Y velocity error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(sum(agent1_common.msg_sent)),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('Agent1 est','\pm 2\sigma','','truth','KF est','Agent2 common est','common \pm 2\sigma')

% agent 2 plots

figure

% x pos
subplot(4,1,1)
hold on; grid on;
plot(input_tvec,x_est_2(5,:) - x_true2_vec(1,:))
plot_xpos_cov_2(:) = sqrt(P_est_2(5,5,:));
plot(input_tvec,2*plot_xpos_cov_2,'r--')
plot(input_tvec,-2*plot_xpos_cov_2,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_base(5,:) - x_true2_vec(1,:),'-.m')
% plot(input_tvec,x_est_1(5,:) - x_true2_vec(1,:))
plot(input_tvec,x_est_1_common(5,:) - x_true2_vec(1,:))
plot(input_tvec,2*sqrt(squeeze(P_est_1_common(5,5,:))'),'g--')
plot(input_tvec,-2*sqrt(squeeze(P_est_1_common(5,5,:))'),'g--')
xlabel('Time [s]')
ylabel('Pos error [m]')
title(['Agent2 est X position error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(sum(agent2_common.msg_sent)),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('Agent2 est','\pm 2\sigma','','truth','KF est','Agent1 common est','common \pm 2\sigma')

% x velocity
subplot(4,1,2)
hold on; grid on;
plot(input_tvec,x_est_2(6,:) - x_true2_vec(2,:))
plot_xvel_cov_2(:) = sqrt(P_est_2(6,6,:));
plot(input_tvec,2*plot_xvel_cov_2,'r--')
plot(input_tvec,-2*plot_xvel_cov_2,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_base(6,:) - x_true2_vec(2,:),'-.m')
% plot(input_tvec,x_est_1(6,:) - x_true2_vec(2,:))
plot(input_tvec,x_est_1_common(6,:) - x_true2_vec(2,:))
plot(input_tvec,2*sqrt(squeeze(P_est_1_common(6,6,:))'),'g--')
plot(input_tvec,-2*sqrt(squeeze(P_est_1_common(6,6,:))'),'g--')
xlabel('Time [s]')
ylabel('Vel error [m/s]')
title(['Agent2 est X velocity error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(sum(agent2_common.msg_sent)),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('Agent2 est','\pm 2\sigma','','truth','KF est','Agent1 common est','common \pm 2\sigma')

subplot(4,1,3)
hold on; grid on;
plot(input_tvec,x_est_2(7,:) - x_true2_vec(3,:))
plot_ypos_cov_2(:) = sqrt(P_est_2(7,7,:));
plot(input_tvec,2*plot_ypos_cov_2,'r--')
plot(input_tvec,-2*plot_ypos_cov_2,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_base(7,:) - x_true2_vec(3,:),'-.m')
% plot(input_tvec,x_est_1(7,:) - x_true2_vec(3,:))
plot(input_tvec,x_est_1_common(7,:) - x_true2_vec(3,:))
plot(input_tvec,2*sqrt(squeeze(P_est_1_common(7,7,:))'),'g--')
plot(input_tvec,-2*sqrt(squeeze(P_est_1_common(7,7,:))'),'g--')
xlabel('Time [s]')
ylabel('Pos error [m]')
title(['Agent2 est Y position error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(sum(agent2_common.msg_sent)),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('Agent2 est','\pm 2\sigma','','truth','KF est','Agent1 common est','common \pm 2\sigma')

subplot(4,1,4)
hold on; grid on;
plot(input_tvec,x_est_2(8,:) - x_true2_vec(4,:))
plot_yvel_cov_2(:) = sqrt(P_est_2(8,8,:));
plot(input_tvec,2*plot_yvel_cov_2,'r--')
plot(input_tvec,-2*plot_yvel_cov_2,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_base(8,:) - x_true2_vec(4,:),'-.m')
% plot(input_tvec,x_est_1(8,:) - x_true2_vec(4,:))
plot(input_tvec,x_est_1_common(8,:) - x_true2_vec(4,:))
plot(input_tvec,2*sqrt(squeeze(P_est_1_common(8,8,:))'),'g--')
plot(input_tvec,-2*sqrt(squeeze(P_est_1_common(8,8,:))'),'g--')
xlabel('Time [s]')
ylabel('Vel error [m/s]')
title(['Agent2 est Y velocity error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(sum(agent2.msg_sent)),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('Agent2 est','\pm 2\sigma','','truth','KF est','Agent1 common est','common \pm 2\sigma')