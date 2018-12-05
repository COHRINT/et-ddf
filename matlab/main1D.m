%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OFFSET 1D Robot Localization w/ Event triggering
%
% Ian Loefgren
% Last Modified: 11.11.2018
%
% main script
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all; clc;

% seed random number generator
rng(100);

%% System Definition

% CT model
A = [0 1;
    0 0];
B = [0; 1];
Gamma = [0; 1];

C = [1 0];
D = 0;

Qct = 1; % additive white gaussian process noise intensity [m/s/s]^2
Rct = 0.5; % additive white gaussian measurement noise intensity [m^2]

% convert to DT model
dt = 0.1;

[F,G,H,M,Q,R] = ct2dt(A,B,C,D,Gamma,Qct,Rct,dt);

% generate input to system
input_tvec = 0:dt:12;
u = 2*cos(0.75*input_tvec);

%% Simulation

% simulate sensor data

% sample intial state
x_t = mvnrnd([0;0]',eye(2))';

for i=1:length(input_tvec)
    
    % simulate process noise
    w = mvnrnd(zeros(1,2),Q)';
    x_true = F*x_t + G*u(i) + w;
    
    % simulate measurement noise
    v = mvnrnd([0],R)';
    y_meas = H*x_true + v;
    
    x_true_vec(:,i) = x_true;
    y_meas_vec(:,i) = y_meas;
    x_t = x_true;

end

% instantiate filters

% create KF object
x0 = [0;0];
P0 = eye(2);
kf = KF(F,G,H,M,Q,R,x0,P0);

% create ETFK object
delta = 4;
etkf = ETKF(F,G,H,M,Q,R,x0,P0,delta);

% run filter
x_pred(:,1) = x0; P_pred(:,:,1) = P0;
x_est(:,1) = x0; P_est(:,:,1) = P0;

x_pred_et(:,1) = x0; P_pred_et(:,:,1) = P0;
x_est_et(:,1) = x0; P_est_et(:,:,1) = P0;

for i=2:length(input_tvec)
    
    [x_curr,P_curr] = kf.predict(u(i));
    x_pred(:,i) = x_curr; P_pred(:,:,i) = P_curr;
    
    [x_curr,P_curr] = kf.update(y_meas_vec(i));
    x_est(:,i) = x_curr; P_est(:,:,i) = P_curr;
    
    [x_curr,P_curr] = etkf.predict(u(i));
    x_pred_et(:,i) = x_curr; P_pred_et(:,:,i) = P_curr;
    
    [x_curr,P_curr] = etkf.update(y_meas_vec(i));
    x_est_et(:,i) = x_curr; P_est_et(:,:,i) = P_curr;
    
end


%% full KF performance
% figure
% 
% subplot(2,1,1)
% hold on; grid on;
% plot(input_tvec,x_est(1,:) - x_true_vec(1,:))
% plot_pos_cov(:) = sqrt(P_est(1,1,:));
% plot(input_tvec,x_est(1,:) - x_true_vec(1,:) + 2*plot_pos_cov,'r--')
% plot(input_tvec,x_est(1,:) - x_true_vec(1,:) - 2*plot_pos_cov,'r--')
% plot(input_tvec,zeros(length(input_tvec),1),'-.k')
% xlabel('Time [s]')
% ylabel('Pos error [m]')
% title('KF est position error and covariance')
% legend('KF est','\pm 2\sigma','','truth')
% 
% subplot(2,1,2)
% hold on; grid on;
% plot(input_tvec,x_est(2,:) - x_true_vec(2,:))
% plot_vel_cov(:) = sqrt(P_est(2,2,:));
% plot(input_tvec,x_est(2,:) - x_true_vec(2,:) + 2*plot_vel_cov,'r--')
% plot(input_tvec,x_est(2,:) - x_true_vec(2,:) - 2*plot_vel_cov,'r--')
% plot(input_tvec,zeros(length(input_tvec),1),'-.k')
% xlabel('Time [s]')
% ylabel('Vel error [m]')
% title('KF est velocity error and covariance')
% legend('KF est','\pm 2\sigma','','truth')


%% event triggered performance
figure

subplot(2,1,1)
hold on; grid on;
plot(input_tvec,x_est_et(1,:) - x_true_vec(1,:))
plot_pos_cov_et(:) = sqrt(P_est_et(1,1,:));
plot(input_tvec,x_est_et(1,:) - x_true_vec(1,:) + 2*plot_pos_cov_et,'r--')
plot(input_tvec,x_est_et(1,:) - x_true_vec(1,:) - 2*plot_pos_cov_et,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_est(1,:) - x_true_vec(1,:),'-.g')
xlabel('Time [s]')
ylabel('Pos error [m]')
title(['ETKF est position error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(etkf.msg_sent),'/',num2str(length(input_tvec)),' msgs'])
legend('ETKF est','\pm 2\sigma','','truth','KF est')

subplot(2,1,2)
hold on; grid on;
plot(input_tvec,x_est_et(2,:) - x_true_vec(2,:))
plot_vel_cov_et(:) = sqrt(P_est_et(2,2,:));
plot(input_tvec,x_est_et(2,:) - x_true_vec(2,:) + 2*plot_vel_cov_et,'r--')
plot(input_tvec,x_est_et(2,:) - x_true_vec(2,:) - 2*plot_vel_cov_et,'r--')
plot(input_tvec,zeros(length(input_tvec),1),'-.k')
plot(input_tvec,x_est(2,:) - x_true_vec(2,:),'-.g')
xlabel('Time [s]')
ylabel('Vel error [m]')
title(['ETKF est velocity error and covariance with \delta=',num2str(delta)...
    ,', ',num2str(etkf.msg_sent),'/',num2str(length(input_tvec)),' msgs'])
legend('ETKF est','\pm 2\sigma','','truth','KF est')



