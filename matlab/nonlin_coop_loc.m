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


x01 = [0 0 0]'; x02 = [0 0 0]';
P01 = diag([1 1 0.001]); P02 = diag([10 10 0.01]);
u1 = [3; 0.2]; u2 = [5; 0.1];

Q1 = diag([1 1 0.01]); Q2 = diag([1 1 0.01]);
noise1 = blkdiag(Q1,zeros(size(u1,1)));
noise2 = blkdiag(Q2,zeros(size(u2,1)));
[t1,y1] = ode45(@(t1,y1) dubin_uni(t1,y1,noise1),[0:dt:tfin],[x01; u1]);
[t2,y2] = ode45(@(t2,y2) dubin_uni(t2,y2,noise2),[0:dt:tfin],[x02; u2]);
y1 = y1'; y2 = y2';

R = [2*pi/180 0; 0 5];

% h = @(x,y) [atan2(y/x);sqrt(x^2 + y^2)] + mvnrnd([0,0],[0.1 0; 0 20])';
Gamma = @(t) [1 0 0; 0 1 0; 0 0 1]; % ********** THIS IS probably NOT CORRECT!!!! ********

% create EKF filter objects
agent1 = EKF(@dubin_uni,@range_bearing,@Atilde,Gamma,@Htilde,Q1,R,dt,x01,P01);
agent2 = EKF(@dubin_uni,@range_bearing,@Atilde,Gamma,@Htilde,Q2,R,dt,x02,P02);


x_est1 = x01;
P_est1 = P01;
meas1(:,1) = [0;0];
x_est2 = x02;
P_est2 = P02;
meas2(:,1) = [0;0];

input_tvec = 0:dt:tfin;
for i=2:length(input_tvec)
    
    % propagate filters
    agent1.predict(i*dt,u1);
    agent2.predict(i*dt,u2);
    
    % update filters
    meas1(:,i) = rel_range_bearing(y1(:,i),y2(:,i),R);
    [x_curr1,P_curr1] = agent1.update(i*dt,meas1(:,i));
    meas2(:,i) = rel_range_bearing(y2(:,i),y1(:,i),R);
    [x_curr2,P_curr2] = agent2.update(i*dt,meas2(:,i));
    
    % save estimate
    x_est1(:,i) = x_curr1; P_est1(:,:,i) = P_curr1;
    x_est2(:,i) = x_curr2; P_est2(:,:,i) = P_curr2;
    
end


figure
hold on; grid on;
plot(y1(1,:),y1(2,:))
plot(x_est1(1,:),x_est1(2,:))
% plot(meas1(2,:).*cos(meas1(1,:)),meas1(2,:).*sin(meas1(1,:)),'x')
plot(y2(1,:),y2(2,:))
plot(x_est2(1,:),x_est2(2,:))
% plot(meas2(2,:).*cos(meas2(1,:)),meas2(2,:).*sin(meas2(1,:)),'x')
legend('truth1','estimate1','measurements1','truth2','estimate2','measurements2')
xlabel('\xi [m]')
ylabel('\eta [m]')
title('EKF estimate over circular trajectory')


figure

subplot(3,1,1)
hold on; grid on;
plot(input_tvec,x_est1(1,:) - y1(1,:))
plot(input_tvec,x_est1(1,:) - y1(1,:) + 2*sqrt(squeeze(P_est1(1,1,:))'),'r--')
plot(input_tvec,x_est1(1,:) - y1(1,:) - 2*sqrt(squeeze(P_est1(1,1,:))'),'r--')
plot(input_tvec,zeros(1,length(input_tvec)),'k-.')
legend('est','\pm 2\sigma','','truth')
xlabel('Time[s]')
ylabel('Est error [m]')
title('\xi est error')

subplot(3,1,2)
hold on; grid on;
plot(input_tvec,x_est1(2,:) - y1(2,:))
plot(input_tvec,x_est1(2,:) - y1(2,:) + 2*sqrt(squeeze(P_est1(2,2,:))'),'r--')
plot(input_tvec,x_est1(2,:) - y1(2,:) - 2*sqrt(squeeze(P_est1(2,2,:))'),'r--')
plot(input_tvec,zeros(1,length(input_tvec)),'k-.')
legend('est','\pm 2\sigma','','truth')
xlabel('Time[s]')
ylabel('Est error [m]')
title('\eta est error')

subplot(3,1,3)
hold on; grid on;
plot(input_tvec,x_est1(3,:) - y1(3,:))
plot(input_tvec,x_est1(3,:) - y1(3,:) + 2*sqrt(squeeze(P_est1(3,3,:))'),'r--')
plot(input_tvec,x_est1(3,:) - y1(3,:) - 2*sqrt(squeeze(P_est1(3,3,:))'),'r--')
plot(input_tvec,zeros(1,length(input_tvec)),'k-.')
legend('est','\pm 2\sigma','','truth')
xlabel('Time[s]')
ylabel('Est error [rad]')
title('\theta est error')


% agent 2 plots
figure

subplot(3,1,1)
hold on; grid on;
plot(input_tvec,x_est2(1,:) - y2(1,:))
plot(input_tvec,x_est2(1,:) - y2(1,:) + 2*sqrt(squeeze(P_est2(1,1,:))'),'r--')
plot(input_tvec,x_est2(1,:) - y2(1,:) - 2*sqrt(squeeze(P_est2(1,1,:))'),'r--')
plot(input_tvec,zeros(1,length(input_tvec)),'k-.')
legend('est','\pm 2\sigma','','truth')
xlabel('Time[s]')
ylabel('Est error [m]')
title('\xi_B est error')

subplot(3,1,2)
hold on; grid on;
plot(input_tvec,x_est2(2,:) - y2(2,:))
plot(input_tvec,x_est2(2,:) - y2(2,:) + 2*sqrt(squeeze(P_est2(2,2,:))'),'r--')
plot(input_tvec,x_est2(2,:) - y2(2,:) - 2*sqrt(squeeze(P_est2(2,2,:))'),'r--')
plot(input_tvec,zeros(1,length(input_tvec)),'k-.')
legend('est','\pm 2\sigma','','truth')
xlabel('Time[s]')
ylabel('Est error [m]')
title('\eta_B est error')

subplot(3,1,3)
hold on; grid on;
plot(input_tvec,x_est2(3,:) - y2(3,:))
plot(input_tvec,x_est2(3,:) - y2(3,:) + 2*sqrt(squeeze(P_est2(3,3,:))'),'r--')
plot(input_tvec,x_est2(3,:) - y2(3,:) - 2*sqrt(squeeze(P_est2(3,3,:))'),'r--')
plot(input_tvec,zeros(1,length(input_tvec)),'k-.')
legend('est','\pm 2\sigma','','truth')
xlabel('Time[s]')
ylabel('Est error [rad]')
title('\theta_B est error')