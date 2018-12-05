%Lec26_1Drobotstatefilter.m
clc,clear

%%fix random number seed to get repeatable results (useful for debugging):
rng(100) %%comment out to get different random results each time


m0 = [0,0];
P0 = eye(2);

% %%%Look at the initial state pdf (multivariate Gaussian)
% [X,Y] = meshgrid([-20:0.1:30],[-10:0.1:10]);
% XY = [X(:),Y(:)];
% pxy0 = reshape( mvnpdf(XY,m0,C0) , size(X)); 
% figure(),
% surf(X,Y,pxy0,'EdgeColor','none'), 
% view(2), xlabel('\xi_0','FontSize',18),ylabel('\xi_0','FontSize',18),
% title('p(\xi_0)','FontSize',18)

%%CT Model spec
A = [0 1;
     0 0];
B = [0;1];
Gamma = [0;1];
C = [1 0];
Qtilde = 1; %AWG process noise intensity: (m/s^2)^2
Rtilde = .5; %AWG measurement noise intensity: m^2
CTsys = ss(A,B,C,0);

%%DT model conversion
deltaT = 0.1
%%Use c2d to cheat/convert parts of model
% %F = [1 deltaT;
% %     0  1];
% %G = [0.5*dt^2;
% %       dt];
% %H = C;
DTsys = c2d(CTsys,deltaT,'zoh');
F = DTsys.a;
G = DTsys.b;
H = DTsys.c;
%%Use Van Loan method to find covariance of process noise
M = deltaT*[-A, Gamma*Qtilde*Gamma';
            zeros(size(A)), A']
matrexp_M = expm(M)        
invFQblock = matrexp_M(1:2,3:4);
Q = F*invFQblock
%%Find the measurement noise covariance
R = Rtilde / deltaT

tvec = 0:deltaT:12;
u = 2*cos(0.75*tvec); %acceleration input

%% 1. Pure forward prediction
%Show recursion for prediction of mean and covariance
mk = m0;
Pk = P0;
mk_pred_hist = zeros(2,length(tvec));
Pk_pred_hist = zeros(2,2,length(tvec));
for k=1:length(tvec)
  
    mkp1 = F*mk' + G*u(k);
    Pkp1 = F*Pk*F' + Q;
    
    %%%pxyk = reshape( mvnpdf(XY,mkp1',Pkp1) , size(X)); %%update mvnpdf
    
    mk = mkp1';
    mk_pred_hist(:,k)=mkp1;
    Pk = Pkp1;
    Pk_pred_hist(:,:,k)=Pkp1;
end

% figure(3), 
% suptitle(['1D Robot State Prediction Results for \Delta t = ',num2str(deltaT)])
% subplot(211)
% plot(tvec,mk_pred_hist(1,:),'b','LineWidth',3), hold on
% plot(tvec,mk_pred_hist(1,:)+2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'b--','LineWidth',2)
% plot(tvec,mk_pred_hist(1,:)-2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'b--','LineWidth',2)
% xlabel('time step, k','FontSize',18), ylabel('\xi_k (m)','FontSize',18),
% legend('mean \xi(k)','2\sigma bounds')
% 
% subplot(212)
% plot(tvec,mk_pred_hist(2,:),'r','LineWidth',3), hold on
% plot(tvec,mk_pred_hist(2,:)+2*sqrt(squeeze(Pk_pred_hist(2,2,:))'),'r--','LineWidth',2)
% plot(tvec,mk_pred_hist(2,:)-2*sqrt(squeeze(Pk_pred_hist(2,2,:))'),'r--','LineWidth',2)
% xlabel('time step, k','FontSize',18), ylabel('dot \xi_k (m/s)','FontSize',18),
% legend('mean dot \xi(k)','2\sigma bounds')

%% 2. Simulate ground truth trajectory and measurements from DT LTI model
xk_truehist = zeros(2,length(tvec));
ykhist = zeros(1,length(tvec));
xk = mvnrnd(m0,P0)'; %sample initial robot state
for k=1:length(tvec)
  
    %%simulate process noise and add to actual state
    wk = mvnrnd(zeros(1,2),Q)';
    xkp1 = F*xk + G*u(k) + wk;     
    
    %%simulate measurement noise and add to sensor data
    vkp1 = mvnrnd(zeros(1,1),R)';
    ykp1 = H*xkp1 + vkp1;
    
    %%store and iterate
    xk_truehist(:,k) = xkp1;
    ykhist(:,k) = ykp1; 
    xk = xkp1;
end


%% 3. Kalman Filter

%%Initialize
mk = m0;
Pk = P0;
mk_filt_hist = zeros(2,length(tvec));
Pk_filt_hist = zeros(2,2,length(tvec));

%%Specify KF's Q and R matrices: 
%%uncomment the appropriate lines below to try out different cases:

%%Case 1: KF uses the correct/true Q and R values for DT system:
Qkf = Q;
Rkf = R;

%%Case 2a: KF assumes smaller Q for DT system, but correct R:
% Qkf = Q*0.001;
% Rkf = R;

%%Case 2b: KF assumes smaller Q for DT system, but correct R:
% Qkf = Q*100;
% Rkf = R;

%%Case 3: KF *thinks* sensors are not as good, but KF uses correct Q
% Qkf = Q;
% Rkf = 10*R; 

%%Apply Kalman filter updates
for k=1:length(tvec)
  
    %%Perform prediction step
    mkp1_minus = F*mk' + G*u(k);
    Pkp1_minus = F*Pk*F' + Qkf;
         
    %%Compute Kalman gain
    Kkp1 = Pkp1_minus*H'/(H*Pkp1_minus*H' + Rkf);
    
    %%Perform measurement update step
    ykp1_report = ykhist(:,k); %pull report of actual data from sensor
    ykp1_pred = H*mkp1_minus; %predicted measurement
    innov_kp1 = ykp1_report - ykp1_pred; %compute innovation
    mkp1_plus = mkp1_minus + Kkp1*innov_kp1; %compute update to state mean
    Pkp1_plus = (eye(2) - Kkp1*H)*Pkp1_minus; %compute update to covar
    
    %%store results and cycle for next iteration
    mk = mkp1_plus'; 
    mk_filt_hist(:,k) = mkp1_plus;
    Pk = Pkp1_plus;
    Pk_filt_hist(:,:,k)=Pkp1_plus;
end

figure(4), 
% suptitle(['1D Robot State Filtering Results for \Delta t = ',num2str(deltaT)])
subplot(211)
plot(tvec,mk_filt_hist(1,:),'b','LineWidth',3), hold on
plot(tvec,mk_filt_hist(1,:)+2*sqrt(squeeze(Pk_filt_hist(1,1,:))'),'b--','LineWidth',2)
plot(tvec,mk_filt_hist(1,:)-2*sqrt(squeeze(Pk_filt_hist(1,1,:))'),'b--','LineWidth',2)
xlabel('time step, k','FontSize',18), ylabel('\xi_k (m)','FontSize',18),
plot(tvec,xk_truehist(1,:),'k-o','LineWidth',1.5)
%%%compare to prediction results
plot(tvec,mk_pred_hist(1,:),'c','LineWidth',1.5), hold on
plot(tvec,mk_pred_hist(1,:)+2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'c--','LineWidth',1.5)
plot(tvec,mk_pred_hist(1,:)-2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'c--','LineWidth',1.5)
ylim([-10 10])
%%plot raw sensor data
plot(tvec,ykhist(1,:),'kx','LineWidth',1.5,'MarkerSize',7)
legend('filter mean \xi(k)','filter 2\sigma bounds','','truth',...
        'mean pred \xi(k)','pred 2\sigma bounds','','y_k sensor data')

subplot(212)
plot(tvec,mk_filt_hist(2,:),'r','LineWidth',3), hold on
plot(tvec,mk_filt_hist(2,:)+2*sqrt(squeeze(Pk_filt_hist(2,2,:))'),'r--','LineWidth',2)
plot(tvec,mk_filt_hist(2,:)-2*sqrt(squeeze(Pk_filt_hist(2,2,:))'),'r--','LineWidth',2)
xlabel('time step, k','FontSize',18), ylabel('dot \xi_k (m/s)','FontSize',18),
legend('mean dot \xi(k)','2\sigma bounds')
plot(tvec,xk_truehist(2,:),'k-o','LineWidth',1.5)
%%%compare to prediction results
plot(tvec,mk_pred_hist(2,:),'m','LineWidth',1.5), hold on
plot(tvec,mk_pred_hist(2,:)+2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'m--','LineWidth',1.5)
plot(tvec,mk_pred_hist(2,:)-2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'m--','LineWidth',1.5)
legend('filter mean dot \xi(k)','filter 2\sigma bounds','','truth',...
        'mean pred dot \xi(k)','pred 2\sigma bounds','')
ylim([-10 10])

%%Plot state estimation errors versus time
figure(5), 
% suptitle(['1D Robot State Estimation Errors for \Delta t = ',num2str(deltaT)])
subplot(211)
plot(tvec,xk_truehist(1,:)-mk_filt_hist(1,:),'b','LineWidth',3), hold on
plot(tvec,xk_truehist(1,:)-mk_filt_hist(1,:)+2*sqrt(squeeze(Pk_filt_hist(1,1,:))'),'b--','LineWidth',2)
plot(tvec,xk_truehist(1,:)-mk_filt_hist(1,:)-2*sqrt(squeeze(Pk_filt_hist(1,1,:))'),'b--','LineWidth',2)
xlabel('time step, k','FontSize',18), ylabel('\xi_k error (m)','FontSize',18),
%%%compare to prediction results
plot(tvec,xk_truehist(1,:)-mk_pred_hist(1,:),'c','LineWidth',1.5), hold on
plot(tvec,xk_truehist(1,:)-mk_pred_hist(1,:)+2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'c--','LineWidth',1.5)
plot(tvec,xk_truehist(1,:)-mk_pred_hist(1,:)-2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'c--','LineWidth',1.5)
plot(tvec,zeros(length(xk_truehist(1,:))),'k-o')
legend('filter error \xi(k)','filter 2\sigma bounds','',...
        'pred error \xi(k)','pred 2\sigma bounds','','truth')
ylim([-12 12])

subplot(212)
plot(tvec,xk_truehist(2,:)-mk_filt_hist(2,:),'r','LineWidth',3), hold on
plot(tvec,xk_truehist(2,:)-mk_filt_hist(2,:)+2*sqrt(squeeze(Pk_filt_hist(2,2,:))'),'r--','LineWidth',2)
plot(tvec,xk_truehist(2,:)-mk_filt_hist(2,:)-2*sqrt(squeeze(Pk_filt_hist(2,2,:))'),'r--','LineWidth',2)
xlabel('time step, k','FontSize',18), ylabel('dot \xi_k error (m/s)','FontSize',18),
%%%compare to prediction results
plot(tvec,xk_truehist(2,:)-mk_pred_hist(2,:),'m','LineWidth',1.5), hold on
plot(tvec,xk_truehist(2,:)-mk_pred_hist(2,:)+2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'m--','LineWidth',1.5)
plot(tvec,xk_truehist(2,:)-mk_pred_hist(2,:)-2*sqrt(squeeze(Pk_pred_hist(1,1,:))'),'m--','LineWidth',1.5)
plot(tvec,zeros(length(xk_truehist(2,:))),'k-o')
legend('filter error dot \xi(k)','filter 2\sigma bounds','',...
        'pred error dot \xi(k)','pred 2\sigma bounds','','truth')
ylim([-12 12])

figure(6)
subplot(211)
plot(tvec,xk_truehist(1,:),'k-o','LineWidth',1.5), hold on
plot(tvec,ykhist(1,:),'rx','LineWidth',1.5,'MarkerSize',7)
legend('true pos','y meas')
xlabel('time step, k','FontSize',18), ylabel('\xi_k (m/s)','FontSize',18),
subplot(212)
plot(tvec,xk_truehist(2,:),'k-o','LineWidth',1.5), hold on
legend('true vel')
xlabel('time step, k','FontSize',18), ylabel('dot \xi_k (m/s)','FontSize',18),