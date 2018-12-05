%Lec29_1Drobotstatefilter.m
%%What happens if we use wrong process noise model and/or dynamics?
%%-->look at NEES and NIS results
%%%CASES: look at
%%%--just ~right: Qkf = Qtrue or close (e.g. ignoring off diagonals)
%%%--too pessimistic: Qkf = diag([0.5 1]) when Wtilde = 1
%%%--too optimisitic: Qkf = diag([5e-6 1e-4]) when Wtilde = 1; 

clc,clear,close all
rng(100)

m0 = [0,0];
P0 = 2*eye(2);


%%True CT Model spec

%%%---Model A: simple NCV kinematics, no velocity drag terms
A = [0 1;
     0 0];
B = [0;1];
Gamma = [0;1];
C = [1 0];
Wtilde = 1; % *ACTUAL* AWG process noise intensity: (m/s^2)^2
Vtilde = 0.005; %AWG measurement noise intensity: m^2

%%%---Model B: NCV kinematics with velocity drag term
% bdrag = -2; 
% A = [0  1;
%      0 bdrag];
% B = [0;1];
% Gamma = [0;1];
% C = [1 0];
% Wtilde = 0.05; % *ACTUAL* AWG process noise intensity: (m/s^2)^2
% Vtilde = 0.005; %AWG measurement noise intensity: m^2


CTsys = ss(A,B,C,0);
%%DT model conversion
deltaT = 0.1

%%Use c2d to cheat/convert parts of model
% %F = [1 deltaT;
% %     0  1];
% %G = [0.5*deltaT^2;
% %       deltaT];
% %H = C;
DTsys = c2d(CTsys,deltaT,'zoh');
F = DTsys.a;
G = DTsys.b;
H = DTsys.c;

%%Use Van Loan method to find covariance of process noise
M = deltaT*[-A, Gamma*Wtilde*Gamma';
            zeros(size(A)), A']
matrexp_M = expm(M)        
invFQblock = matrexp_M(1:2,3:4);
Qtrue = F*invFQblock
%%Find the measurement noise covariance
Rtrue = Vtilde / deltaT


%%Open-loop Control input
tvec = 0:deltaT:12;
u = 2*cos(0.75*tvec); %acceleration input

%% Run chi-square tests on simulated filtering runs:
%%Specify KF model of dynamics:
%%% *----* MAY BE MISMATCHED TO ACTUAL SYSTEM MODEL !!! *----*%%%
Fkf = [1 deltaT;
    0     1];
Gkf =  [0.5*deltaT^2;
        deltaT];
Hkf = [1 0];

%%%***Specify assumed DT noise covariances for KF:
%%%MAY BE MISMATCHED TO ACTUAL VALUES !!! ***
Qkf = diag([5e-3 1e-3 ]);% diag([0.5 1])% Qtrue;%  %%treat this as the main tuning knob
Rkf = Rtrue; %%assume we can get this from sensor specs

Nsimruns = 50
NEESsamps = zeros(Nsimruns,length(tvec));
NISsamps = zeros(Nsimruns,length(tvec));

for ss=1:Nsimruns
    %%%1. %%Generate true trajectory and measurements from system
    
    xk_truehist = zeros(2,length(tvec));
    ykhist = zeros(1,length(tvec));
    xk = mvnrnd(m0,P0); %sample initial robot state
    for jj=1:length(tvec)
        
        wk = mvnrnd(zeros(1,2),Qtrue);
        xkp1 = F*xk' + G*u(jj) + wk';
        vkp1 = mvnrnd(zeros(1,1),Rtrue)';
        ykp1 = H*xkp1 + vkp1;
        
        xk_truehist(:,jj) = xkp1;
        ykhist(:,jj) = ykp1;
        xk = xkp1';
    end
    
    %%% 2. Kalman Filter equations with simple NCV model
    
    %%Run the Kalman filter updates
    mk = m0;
    Pk = P0;
    mk_filt_hist = zeros(2,length(tvec));
    Pk_filt_hist = zeros(2,2,length(tvec));
    innovk_hist = zeros(1,length(tvec));
    Pyyk_hist = zeros(1,1,length(tvec)); %%store measurement innovation covar
    NEESsshist = zeros(1,length(tvec));
    NISsshist = zeros(1,length(tvec));
    for jj=1:length(tvec)
        
        %%Perform prediction step
        mkp1_minus = F*mk' + G*u(jj);
        Pkp1_minus = F*Pk*F' + Qkf;
        
        %%Compute Kalman gain
        Pyykp1 = H*Pkp1_minus*H' + Rtrue;
        Pyykp1 = 0.5*(Pyykp1 + Pyykp1');
        Kkp1 = Pkp1_minus*H'/(H*Pkp1_minus*H' + Rtrue);
        %%Perform measurement update step
        ykp1_report = ykhist(:,jj); %simulate the reporting of data from sensor
        ykp1_pred = H*mkp1_minus; %predicted measurement
        innov_kp1 = ykp1_report - ykp1_pred; %compute meas innovation
        mkp1_plus = mkp1_minus + Kkp1*innov_kp1; %compute update to state mean
        Pkp1_plus = (eye(2) - Kkp1*H)*Pkp1_minus; %compute update to covar
        
        mk = mkp1_plus';
        mk_filt_hist(:,jj) = mkp1_plus;
        Pk = Pkp1_plus;
        Pk_filt_hist(:,:,jj)= Pkp1_plus;
        innovk_hist(:,jj) = innov_kp1;
        
        %%Compute and store NEES and NIS statistics:
        invPkp1 = inv(Pkp1_plus);
        invPyykp1 = inv(Pyykp1);
        NEESsshist(jj) = ...
            (xk_truehist(:,jj) - mkp1_plus)'*invPkp1*(xk_truehist(:,jj) - mkp1_plus);
        NISsshist(jj) = innov_kp1'*invPyykp1*innov_kp1;                
    end
    NEESsamps(ss,:) = NEESsshist;
    NISsamps(ss,:) = NISsshist;
    
    %%sanity check:
    %Plot state estimation errors versus time
    figure(5),
    clf
    subplot(211)
    plot(tvec,xk_truehist(1,:)-mk_filt_hist(1,:),'b','LineWidth',3), hold on
    plot(tvec,2*sqrt(squeeze(Pk_filt_hist(1,1,:))'),'b--','LineWidth',2)
    plot(tvec,-2*sqrt(squeeze(Pk_filt_hist(1,1,:))'),'b--','LineWidth',2)
    subplot(212)
    plot(tvec,xk_truehist(2,:)-mk_filt_hist(2,:),'b','LineWidth',3), hold on
    plot(tvec,2*sqrt(squeeze(Pk_filt_hist(2,2,:))'),'b--','LineWidth',2)
    plot(tvec,-2*sqrt(squeeze(Pk_filt_hist(2,2,:))'),'b--','LineWidth',2)
    
end

%%DO NEES TEST:
epsNEESbar = mean(NEESsamps,1);
alphaNEES = 0.05; %%significance level
Nnx = Nsimruns*length(Fkf);
%%compute intervals:
r1x = chi2inv(alphaNEES/2, Nnx )./ Nsimruns
r2x = chi2inv(1-alphaNEES/2, Nnx )./ Nsimruns
figure(80)
plot(epsNEESbar,'ro','MarkerSize',6,'LineWidth',2),hold on
plot(r1x*ones(size(epsNEESbar)),'r--','LineWidth',2)
plot(r2x*ones(size(epsNEESbar)),'r--','LineWidth',2)
ylabel('NEES statistic, \bar{\epsilon}_x','FontSize',14)
xlabel('time step, k','FontSize',14)
title('NEES Estimation Results','FontSize',14)
legend('NEES @ time k', 'r_1 bound', 'r_2 bound')

%%DO NIS TEST:
epsNISbar = mean(NISsamps,1);
alphaNIS = 0.05; 
Nny = Nsimruns*size(H,1);
%%compute intervals:
r1y = chi2inv(alphaNIS/2, Nny )./ Nsimruns
r2y = chi2inv(1-alphaNIS/2, Nny )./ Nsimruns
figure(90)
plot(epsNISbar,'bo','MarkerSize',6,'LineWidth',2),hold on
plot(r1y*ones(size(epsNISbar)),'b--','LineWidth',2)
plot(r2y*ones(size(epsNISbar)),'b--','LineWidth',2)
ylabel('NIS statistic, \bar{\epsilon}_y','FontSize',14)
xlabel('time step, k','FontSize',14)
title('NIS Estimation Results','FontSize',14)
legend('NIS @ time k', 'r_1 bound', 'r_2 bound')

% figure(1), 
% suptitle(['1D Robot State Filtering Results for \Delta t = ',num2str(deltaT)])
% subplot(211)
% plot(tvec,mk_filt_hist(1,:),'b','LineWidth',3), hold on
% plot(tvec,mk_filt_hist(1,:)+2*sqrt(squeeze(Pk_filt_hist(1,1,:))'),'b--','LineWidth',2)
% plot(tvec,mk_filt_hist(1,:)-2*sqrt(squeeze(Pk_filt_hist(1,1,:))'),'b--','LineWidth',2)
% xlabel('time step, k','FontSize',18), ylabel('\xi_k (m)','FontSize',18),
% plot(tvec,xk_truehist(1,:),'k-o','LineWidth',1.5)
% %%%compare to prediction results
% plot(tvec,mk_pred_hist(1,:),'c','LineWidth',1.5), hold on
% plot(tvec,mk_pred_hist(1,:)+2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'c--','LineWidth',1.5)
% plot(tvec,mk_pred_hist(1,:)-2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'c--','LineWidth',1.5)
% ylim([-10 10])
% %%plot raw sensor data
% plot(tvec,ykhist(1,:),'kx','LineWidth',1.5,'MarkerSize',7)
% legend('filter mean \xi(k)','filter 2\sigma bounds','','truth',...
%         'mean pred \xi(k)','pred 2\sigma bounds','','y_k sensor data')
% 
% subplot(212)
% plot(tvec,mk_filt_hist(2,:),'r','LineWidth',3), hold on
% plot(tvec,mk_filt_hist(2,:)+2*sqrt(squeeze(Pk_filt_hist(2,2,:))'),'r--','LineWidth',2)
% plot(tvec,mk_filt_hist(2,:)-2*sqrt(squeeze(Pk_filt_hist(2,2,:))'),'r--','LineWidth',2)
% xlabel('time step, k','FontSize',18), ylabel('dot \xi_k (m/s)','FontSize',18),
% legend('mean dot \xi(k)','2\sigma bounds')
% plot(tvec,xk_truehist(2,:),'k-o','LineWidth',1.5)
% %%%compare to prediction results
% plot(tvec,mk_pred_hist(2,:),'m','LineWidth',1.5), hold on
% plot(tvec,mk_pred_hist(2,:)+2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'m--','LineWidth',1.5)
% plot(tvec,mk_pred_hist(2,:)-2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'m--','LineWidth',1.5)
% legend('filter mean dot \xi(k)','filter 2\sigma bounds','','truth',...
%         'mean pred dot \xi(k)','pred 2\sigma bounds','')
% ylim([-10 10])
% 
% %%Plot state estimation errors versus time
% figure(5), 
% suptitle(['1D Robot State Estimation Errors for \Delta t = ',num2str(deltaT)])
% subplot(211)
% plot(tvec,xk_truehist(1,:)-mk_filt_hist(1,:),'b','LineWidth',3), hold on
% plot(tvec,xk_truehist(1,:)-mk_filt_hist(1,:)+2*sqrt(squeeze(Pk_filt_hist(1,1,:))'),'b--','LineWidth',2)
% plot(tvec,xk_truehist(1,:)-mk_filt_hist(1,:)-2*sqrt(squeeze(Pk_filt_hist(1,1,:))'),'b--','LineWidth',2)
% xlabel('time step, k','FontSize',18), ylabel('\xi_k error (m)','FontSize',18),
% %%%compare to prediction results
% plot(tvec,xk_truehist(1,:)-mk_pred_hist(1,:),'c','LineWidth',1.5), hold on
% plot(tvec,xk_truehist(1,:)-mk_pred_hist(1,:)+2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'c--','LineWidth',1.5)
% plot(tvec,xk_truehist(1,:)-mk_pred_hist(1,:)-2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'c--','LineWidth',1.5)
% plot(tvec,zeros(length(xk_truehist(1,:))),'k-o')
% legend('filter error \xi(k)','filter 2\sigma bounds','',...
%         'pred error \xi(k)','pred 2\sigma bounds','','truth')
% ylim([-12 12])
% 
% subplot(212)
% plot(tvec,xk_truehist(2,:)-mk_filt_hist(2,:),'r','LineWidth',3), hold on
% plot(tvec,xk_truehist(2,:)-mk_filt_hist(2,:)+2*sqrt(squeeze(Pk_filt_hist(2,2,:))'),'r--','LineWidth',2)
% plot(tvec,xk_truehist(2,:)-mk_filt_hist(2,:)-2*sqrt(squeeze(Pk_filt_hist(2,2,:))'),'r--','LineWidth',2)
% xlabel('time step, k','FontSize',18), ylabel('dot \xi_k error (m/s)','FontSize',18),
% %%%compare to prediction results
% plot(tvec,xk_truehist(2,:)-mk_pred_hist(2,:),'m','LineWidth',1.5), hold on
% plot(tvec,xk_truehist(2,:)-mk_pred_hist(2,:)+2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'m--','LineWidth',1.5)
% plot(tvec,xk_truehist(2,:)-mk_pred_hist(2,:)-2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'m--','LineWidth',1.5)
% plot(tvec,zeros(length(xk_truehist(2,:))),'k-o')
% legend('filter error dot \xi(k)','filter 2\sigma bounds','',...
%         'pred error dot \xi(k)','pred 2\sigma bounds','','truth')
% ylim([-12 12])
% 
% figure(6)
% subplot(211)
% plot(tvec,xk_truehist(1,:),'k-o','LineWidth',1.5), hold on
% plot(tvec,ykhist(1,:),'rx','LineWidth',1.5,'MarkerSize',7)
% legend('true pos','y meas')
% xlabel('time step, k','FontSize',18), ylabel('\xi_k (m/s)','FontSize',18),
% subplot(212)
% plot(tvec,xk_truehist(2,:),'k-o','LineWidth',1.5), hold on
% legend('true vel')
% xlabel('time step, k','FontSize',18), ylabel('dot \xi_k (m/s)','FontSize',18),