%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OFFSET 2D Robot Localization w/ Event triggering
%
% Ian Loefgren
% Last Modified: 11.13.2018
%
% main script
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all; clc;

% seed random number generator
rng(100);

%% System Definition

obs2pos = [10;20]; % static position of second observer

% CT model
A = [0 1 0 0;
    0 0 0 0;
    0 0 0 1;
    0 0 0 0];
B = [0 0;
    1 0;
    0 0;
    0 1];
Gamma = [0 0;
         1 0;
         0 0;
         0 1];

C = [1 0 0 0;
    0 0 1 0];

D = 0;

Qct = 5; % additive white gaussian process noise intensity [m/s/s]^2
Rct = 1; % additive white gaussian measurement noise intensity [m^2]

% convert to DT model
dt = 0.1;

[F,G,H,M,Q,R] = ct2dt(A,B,C,D,Gamma,Qct,Rct,dt);

% generate input to system
input_tvec = 0:dt:12;
u1 = 2*cos(0.75*input_tvec);
u2 = 2*sin(input_tvec);
u = [u1;u2];

%% Simulation

% simulate sensor data
x_t = mvnrnd(zeros(1,4),eye(4))';

for i=1:length(input_tvec)
    
    % simulate process noise
    w = mvnrnd(zeros(1,4),Q)';
    x_true = F*x_t + G*u(:,i) + w;
    
    % simulate measurement noise
    v = mvnrnd(zeros(1,2),R)';
    y_meas_obs1 = H*x_true + v;
    v2 = mvnrnd(zeros(1,2),R)';
    y_meas_obs2 = H*x_true - obs2pos + v2;
    
    x_true_vec(:,i) = x_true;
    y_meas_vec_1(:,i) = y_meas_obs1;
    y_meas_vec_2(:,i) = y_meas_obs2;
    x_t = x_true;
    
end

% instantiate filters

% create KF object
x0 = zeros(4,1);
P0 = eye(4);
kf = KF(F,G,H,M,Q,R,x0,P0);

% create ETFK observer objects
delta = 0;
R = [5 0; 0 40];
obs1 = ETKF(F,G,H,M,Q,R,x0,P0,delta);
R = [20 0; 0 5];
obs2 = ETKF(F,G,H,M,Q,R,x0,P0,delta);

% run filters
x_pred(:,1) = x0; P_pred(:,:,1) = P0;
x_est(:,1) = x0; P_est(:,:,1) = P0;

x_pred_et(:,1) = x0; P_pred_et(:,:,1) = P0;
x_est_et(:,1) = x0; P_est_et(:,:,1) = P0;

x_pred_et_obs1(:,1) = x0; P_pred_et_obs1(:,:,1) = P0;
x_est_et_obs1(:,1) = x0; P_est_et_obs1(:,:,1) = P0;
x_pred_et_obs2(:,1) = x0; P_pred_et_obs2(:,:,1) = P0;
x_est_et_obs2(:,1) = x0; P_est_et_obs2(:,:,1) = P0;

for i=2:length(input_tvec)
    
    [x_curr,P_curr] = kf.predict(u(:,i));
    x_pred(:,i) = x_curr; P_pred(:,:,i) = P_curr;
    
    [x_curr,P_curr] = kf.update(y_meas_vec_1(:,i));
    x_est(:,i) = x_curr; P_est(:,:,i) = P_curr;
    
    [x_curr,P_curr] = obs1.predict(u(:,i));
    x_pred_et_obs1(:,i) = x_curr; P_pred_et_obs1(:,:,i) = P_curr;
    
    [x_curr,P_curr] = obs1.update(y_meas_vec_1(:,i));
    x_est_et_obs1(:,i) = x_curr; P_est_et_obs1(:,:,i) = P_curr;
    
    [x_curr,P_curr] = obs2.predict(u(:,i));
    x_pred_et_obs2(:,i) = x_curr; P_pred_et_obs2(:,:,i) = P_curr;
    
    [x_curr,P_curr] = obs2.update(y_meas_vec_2(:,i)+obs2pos);
    x_est_et_obs2(:,i) = x_curr; P_est_et_obs2(:,:,i) = P_curr;
end

%% full KF performance
% figure
% 
% subplot(2,2,1)
% hold on; grid on;
% plot(input_tvec,x_est(1,:) - x_true_vec(1,:))
% plot_xpos_cov(:) = sqrt(P_est(1,1,:));
% plot(input_tvec,x_est(1,:) - x_true_vec(1,:) + 2*plot_xpos_cov,'r--')
% plot(input_tvec,x_est(1,:) - x_true_vec(1,:) - 2*plot_xpos_cov,'r--')
% plot(input_tvec,zeros(length(input_tvec),1),'-.k')
% xlabel('Time [s]')
% ylabel('X Pos error [m]')
% title('KF est X position error and covariance')
% legend('KF est','\pm 2\sigma','','truth')
% 
% subplot(2,2,2)
% hold on; grid on;
% plot(input_tvec,x_est(2,:) - x_true_vec(2,:))
% plot_xvel_cov(:) = sqrt(P_est(2,2,:));
% plot(input_tvec,x_est(2,:) - x_true_vec(2,:) + 2*plot_xvel_cov,'r--')
% plot(input_tvec,x_est(2,:) - x_true_vec(2,:) - 2*plot_xvel_cov,'r--')
% plot(input_tvec,zeros(length(input_tvec),1),'-.k')
% xlabel('Time [s]')
% ylabel('X Vel error [m]')
% title('KF est X velocity error and covariance')
% legend('KF est','\pm 2\sigma','','truth')
% 
% subplot(2,2,3)
% hold on; grid on;
% plot(input_tvec,x_est(3,:) - x_true_vec(3,:))
% plot_ypos_cov(:) = sqrt(P_est(3,3,:));
% plot(input_tvec,x_est(3,:) - x_true_vec(3,:) + 2*plot_ypos_cov,'r--')
% plot(input_tvec,x_est(3,:) - x_true_vec(3,:) - 2*plot_ypos_cov,'r--')
% plot(input_tvec,zeros(length(input_tvec),1),'-.k')
% xlabel('Time [s]')
% ylabel('Y Pos error [m]')
% title('KF est Y position error and covariance')
% legend('KF est','\pm 2\sigma','','truth')
% 
% subplot(2,2,4)
% hold on; grid on;
% plot(input_tvec,x_est(4,:) - x_true_vec(4,:))
% plot_yvel_cov(:) = sqrt(P_est(4,4,:));
% plot(input_tvec,x_est(4,:) - x_true_vec(4,:) + 2*plot_yvel_cov,'r--')
% plot(input_tvec,x_est(4,:) - x_true_vec(4,:) - 2*plot_yvel_cov,'r--')
% plot(input_tvec,zeros(length(input_tvec),1),'-.k')
% xlabel('Time [s]')
% ylabel('Y Vel error [m]')
% title('KF est Y velocity error and covariance')
% legend('KF est','\pm 2\sigma','','truth')
% 

% plot 2D trajectory, each observer estimate and select individual and
% fused covariances
figure
hold on; grid on;
plot(x_true_vec(1,:),x_true_vec(3,:),'k')
plot(x_est_et_obs1(1,:),x_est_et_obs1(3,:))
plot(x_est_et_obs2(1,:),x_est_et_obs2(3,:))
for i=1:10:length(x_est(1,:))
    error_ellipse(squeeze(P_est_et_obs1([1,3],[1,3],i)),[x_est_et_obs1(1,i), x_est_et_obs1(3,i)]);
    error_ellipse(squeeze(P_est_et_obs2([1,3],[1,3],i)),[x_est_et_obs2(1,i), x_est_et_obs2(3,i)]);
    [xc,Pc] = covar_intersect([x_est_et_obs1(1,i); x_est_et_obs1(3,i)],...
                                [x_est_et_obs2(1,i); x_est_et_obs2(3,i)],...
                                squeeze(P_est_et_obs1([1,3],[1,3],i)),...
                                squeeze(P_est_et_obs2([1,3],[1,3],i)));
    error_ellipse(Pc,xc);
end
axis equal


%% event triggered performance
figure

subplot(2,2,1)
hold on; grid on;
plot(input_tvec,x_est_et_obs1(1,:) - x_true_vec(1,:))
plot_xpos_cov_et(:) = sqrt(P_est_et_obs1(1,1,:));
plot(input_tvec,x_est_et_obs1(1,:) - x_true_vec(1,:) + 2*plot_xpos_cov_et,'r--')
plot(input_tvec,x_est_et_obs1(1,:) - x_true_vec(1,:) - 2*plot_xpos_cov_et,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_est(1,:) - x_true_vec(1,:),'-.g')
xlabel('Time [s]')
ylabel('Pos error [m]')
title(['ETKF est X position error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(obs1.msg_sent),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('ETKF est','\pm 2\sigma','','truth','KF est')

subplot(2,2,2)
hold on; grid on;
plot(input_tvec,x_est_et_obs1(2,:) - x_true_vec(2,:))
plot_xvel_cov_et(:) = sqrt(P_est_et_obs1(2,2,:));
plot(input_tvec,x_est_et_obs1(2,:) - x_true_vec(2,:) + 2*plot_xvel_cov_et,'r--')
plot(input_tvec,x_est_et_obs1(2,:) - x_true_vec(2,:) - 2*plot_xvel_cov_et,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_est(2,:) - x_true_vec(2,:),'-.g')
xlabel('Time [s]')
ylabel('Vel error [m]')
title(['ETKF est X velocity error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(obs1.msg_sent),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('ETKF est','\pm 2\sigma','','truth','KF est')

subplot(2,2,3)
hold on; grid on;
plot(input_tvec,x_est_et_obs1(3,:) - x_true_vec(3,:))
plot_ypos_cov_et(:) = sqrt(P_est_et_obs1(3,3,:));
plot(input_tvec,x_est_et_obs1(3,:) - x_true_vec(3,:) + 2*plot_ypos_cov_et,'r--')
plot(input_tvec,x_est_et_obs1(3,:) - x_true_vec(3,:) - 2*plot_ypos_cov_et,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_est(3,:) - x_true_vec(3,:),'-.g')
xlabel('Time [s]')
ylabel('Pos error [m]')
title(['ETKF est Y position error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(obs1.msg_sent),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('ETKF est','\pm 2\sigma','','truth','KF est')

subplot(2,2,4)
hold on; grid on;
plot(input_tvec,x_est_et_obs1(4,:) - x_true_vec(4,:))
plot_yvel_cov_et(:) = sqrt(P_est_et_obs1(4,4,:));
plot(input_tvec,x_est_et_obs1(4,:) - x_true_vec(4,:) + 2*plot_yvel_cov_et,'r--')
plot(input_tvec,x_est_et_obs1(4,:) - x_true_vec(4,:) - 2*plot_yvel_cov_et,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_est(4,:) - x_true_vec(4,:),'-.g')
xlabel('Time [s]')
ylabel('Vel error [m]')
title(['ETKF est Y velocity error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(obs1.msg_sent),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('ETKF est','\pm 2\sigma','','truth','KF est')

figure

subplot(2,2,1)
hold on; grid on;
plot(input_tvec,x_est_et_obs2(1,:) - x_true_vec(1,:))
plot_xpos_cov_et(:) = sqrt(P_est_et_obs2(1,1,:));
plot(input_tvec,x_est_et_obs2(1,:) - x_true_vec(1,:) + 2*plot_xpos_cov_et,'r--')
plot(input_tvec,x_est_et_obs2(1,:) - x_true_vec(1,:) - 2*plot_xpos_cov_et,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_est(1,:) - x_true_vec(1,:),'-.g')
xlabel('Time [s]')
ylabel('Pos error [m]')
title(['Obs2 ETKF est X position error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(obs2.msg_sent),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('ETKF est','\pm 2\sigma','','truth','KF est')

subplot(2,2,2)
hold on; grid on;
plot(input_tvec,x_est_et_obs2(2,:) - x_true_vec(2,:))
plot_xvel_cov_et(:) = sqrt(P_est_et_obs2(2,2,:));
plot(input_tvec,x_est_et_obs2(2,:) - x_true_vec(2,:) + 2*plot_xvel_cov_et,'r--')
plot(input_tvec,x_est_et_obs2(2,:) - x_true_vec(2,:) - 2*plot_xvel_cov_et,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_est(2,:) - x_true_vec(2,:),'-.g')
xlabel('Time [s]')
ylabel('Vel error [m]')
title(['ETKF est X velocity error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(obs2.msg_sent),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('ETKF est','\pm 2\sigma','','truth','KF est')

subplot(2,2,3)
hold on; grid on;
plot(input_tvec,x_est_et_obs2(3,:) - x_true_vec(3,:))
plot_ypos_cov_et(:) = sqrt(P_est_et_obs2(3,3,:));
plot(input_tvec,x_est_et_obs2(3,:) - x_true_vec(3,:) + 2*plot_ypos_cov_et,'r--')
plot(input_tvec,x_est_et_obs2(3,:) - x_true_vec(3,:) - 2*plot_ypos_cov_et,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_est(3,:) - x_true_vec(3,:),'-.g')
xlabel('Time [s]')
ylabel('Pos error [m]')
title(['ETKF est Y position error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(obs2.msg_sent),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('ETKF est','\pm 2\sigma','','truth','KF est')

subplot(2,2,4)
hold on; grid on;
plot(input_tvec,x_est_et_obs2(4,:) - x_true_vec(4,:))
plot_yvel_cov_et(:) = sqrt(P_est_et_obs2(4,4,:));
plot(input_tvec,x_est_et_obs2(4,:) - x_true_vec(4,:) + 2*plot_yvel_cov_et,'r--')
plot(input_tvec,x_est_et_obs2(4,:) - x_true_vec(4,:) - 2*plot_yvel_cov_et,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_est(4,:) - x_true_vec(4,:),'-.g')
xlabel('Time [s]')
ylabel('Vel error [m]')
title(['ETKF est Y velocity error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(obs2.msg_sent),'/',num2str(length(input_tvec)*size(H,1)),' msgs'])
legend('ETKF est','\pm 2\sigma','','truth','KF est')
