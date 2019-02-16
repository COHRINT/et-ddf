% OFFSET swarm localization Regression
%
% Ian Loefgren
% 2.15.2019
%

clear; clc; close all;

% load cost data
load('cost.mat')

% unpack data
delta_vec = cost(:,2);
tau_vec = cost(:,3);
covar_vec = cost(:,4);
data_trans_vec = cost(:,5);
cost_val_vec = cost(:,6);


%% plot raw data

% figure
% hold on; grid on;
% plot3(delta_vec,tau_vec,covar_vec,'x')
% xlabel('\delta')
% ylabel('\tau (per state)')
% zlabel('avg. covariance trace')
% 
% figure
% hold on; grid on;
% plot3(delta_vec,tau_vec,data_trans_vec,'x');
% xlabel('\delta')
% ylabel('\tau (per state)')
% zlabel('avg. data transfer per agent')
% 
% figure
% hold on; grid on;
% plot3(delta_vec,tau_vec,cost_val_vec,'x');
% xlabel('\delta')
% ylabel('\tau (per state)')
% zlabel('cost')

% try matlab polynomial surface fit
% f = fit([delta_vec,tau_vec],covar_vec,'poly2')

[X,Y] = meshgrid(0:0.1:5,0:0.1:20);
X_vec = reshape(X,[size(X,1)*size(X,2) 1]);
Y_vec = reshape(Y,[size(Y,1)*size(Y,2) 1]);

% gaussian process regression
f_covar_gp = fitrgp([delta_vec,tau_vec],covar_vec);
covar_pred = resubPredict(f_covar_gp);
[covar_grid,~,sigma_1] = predict(f_covar_gp,[X_vec,Y_vec]);
covar_grid = reshape(covar_grid,size(X));

f_data_gp = fitrgp([delta_vec,tau_vec],data_trans_vec);
data_trans_pred = resubPredict(f_data_gp);
[data_trans_grid,~,sigma_2] = predict(f_data_gp,[X_vec Y_vec]);
data_trans_grid = reshape(data_trans_grid,size(X));

f_cost_gp = fitrgp([delta_vec,tau_vec],cost_val_vec);
cost_pred = resubPredict(f_cost_gp);
[cost_grid,~,sigma_3] = predict(f_cost_gp,[X_vec,Y_vec]);
cost_grid = reshape(cost_grid,size(X));

figure
hold on; grid on;
plot3(delta_vec,tau_vec,covar_vec,'rx');
s1 = surf(X,Y,covar_grid,'FaceAlpha',0.75);
s1.EdgeColor = 'none';
xlabel('\delta')
ylabel('\tau (per state)')
zlabel('avg. covariance trace')
legend('data','gp fit pred')
title('covar data and fit')

figure
hold on; grid on;
plot3(delta_vec,tau_vec,data_trans_vec,'x');
s1 = surf(X,Y,data_trans_grid,'FaceAlpha',0.75);
s1.EdgeColor = 'none';
xlabel('\delta')
ylabel('\tau (per state)')
zlabel('avg. agent data trans')
legend('data','gp fit pred')
title('data trans data and fit')

figure
hold on; grid on;
plot3(delta_vec,tau_vec,cost_val_vec,'x');
s1 = surf(X,Y,cost_grid,'FaceAlpha',0.75);
s1.EdgeColor = 'none';
xlabel('\delta')
ylabel('\tau (per state)')
zlabel('cost')
legend('data','gp fit pred')
title('cost data and fit')

