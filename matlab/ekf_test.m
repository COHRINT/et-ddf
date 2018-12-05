% Non-linear Cooperative Localization
%
% 12.1.2018
%
%

clear; close all; clc;

rng(6969);

% define two agents modeled by dubin's unicycles

dt = 0.1;
tfin = 30;


x0 = [0 0 0]';
P0 = diag([1 1 0.001]);
u = [3; 0.2];

Q = diag([1 1 0.01]);
noise = blkdiag(Q,zeros(size(u,1)));
[t1,y1] = ode45(@(t,y) dubin_uni(t,y,noise),[0:dt:tfin],[x0; u]);
[t2,y2] = ode45(@(t,y) dubin_uni(t,y,0),[0:dt:tfin],[x0; u]);
y1 = y1'; y2 = y2';

R = [2*pi/180 0; 0 5];

% h = @(x,y) [atan2(y/x);sqrt(x^2 + y^2)] + mvnrnd([0,0],[0.1 0; 0 20])';
Gamma = @(t) [1 0 0; 0 1 0; 0 0 1]; % ********** THIS IS probably NOT CORRECT!!!! ********

% create EKF filter object
filter = EKF(@dubin_uni,@range_bearing,@Atilde,Gamma,@Htilde,Q,R,dt,x0,P0);


x_est = x0;
P_est = P0;
meas(:,1) = [0;0];

input_tvec = 0:dt:tfin;
for i=2:length(input_tvec)
    % propagate filter
    filter.predict(i*dt,u);
    
    % update filter
    meas(:,i) = range_bearing(y1(:,i),R);
    [x_curr,P_curr] = filter.update(i*dt,meas(:,i));
    
    % save estimate
    x_est(:,i) = x_curr; P_est(:,:,i) = P_curr;
    
end


figure
hold on; grid on;
plot(y1(1,:),y1(2,:))
% plot(y2(2,:),y2(2,:))
plot(x_est(1,:),x_est(2,:))
plot(meas(2,:).*cos(meas(1,:)),meas(2,:).*sin(meas(1,:)),'x')
legend('truth','estimate','measurements')
xlabel('\xi [m]')
ylabel('\eta [m]')
title('EKF estimate over circular trajectory')


figure

subplot(3,1,1)
hold on; grid on;
plot(input_tvec,x_est(1,:) - y1(1,:))
plot(input_tvec,x_est(1,:) - y1(1,:) + 2*sqrt(squeeze(P_est(1,1,:))'),'r--')
plot(input_tvec,x_est(1,:) - y1(1,:) - 2*sqrt(squeeze(P_est(1,1,:))'),'r--')
plot(input_tvec,zeros(1,length(input_tvec)),'k-.')
legend('est','\pm 2\sigma','','truth')
xlabel('Time[s]')
ylabel('Est error [m]')
title('\xi est error')

subplot(3,1,2)
hold on; grid on;
plot(input_tvec,x_est(2,:) - y1(2,:))
plot(input_tvec,x_est(2,:) - y1(2,:) + 2*sqrt(squeeze(P_est(2,2,:))'),'r--')
plot(input_tvec,x_est(2,:) - y1(2,:) - 2*sqrt(squeeze(P_est(2,2,:))'),'r--')
plot(input_tvec,zeros(1,length(input_tvec)),'k-.')
legend('est','\pm 2\sigma','','truth')
xlabel('Time[s]')
ylabel('Est error [m]')
title('\eta est error')

subplot(3,1,3)
hold on; grid on;
plot(input_tvec,x_est(3,:) - y1(3,:))
plot(input_tvec,x_est(3,:) - y1(3,:) + 2*sqrt(squeeze(P_est(3,3,:))'),'r--')
plot(input_tvec,x_est(3,:) - y1(3,:) - 2*sqrt(squeeze(P_est(3,3,:))'),'r--')
plot(input_tvec,zeros(1,length(input_tvec)),'k-.')
legend('est','\pm 2\sigma','','truth')
xlabel('Time[s]')
ylabel('Est error [rad]')
title('\theta est error')