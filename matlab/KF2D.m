% 2D Kalman filter implementation with implicit measurements
% 
% Robot can travel on a line, can take measurements, and also receive
% meaurements from a ranging device at the origin

clear; close all; clc;

% state vector
% x = [x xdot]
rng(965);
animation = false;
plots = true;

% number of robots
num_robots = 2;

% timespan
tspan = [0 50]; %[s]
% time vector
dt = 0.1;
t = tspan(1):dt:tspan(2);


% event triggering threshold
delta = 0.000001;
% implicit measurement pdf fxn handle
phi = @(z) (1/sqrt(2*pi)) * exp(-0.5*z^2);
% Qfxn = @(z) 0.5*erfc(-z/sqrt(2));
Qfxn = @(z) normcdf(z,0,1,'upper');

% dynamics

% STM
Phi = [1 0 dt 0;
        0 1 0 dt;
        0 0 1 0;
        0 0 0 1];
    
%% true trajectory
% initial conditions
x0 = [0; 0; 0.05; 0.05];
xtrue = zeros(4,length(t));
xtrue(:,1) = x0;
    


%% KF

% initial conditions
x0 = [0; 0; 0.1; 0.1];
P0 = [5 0 0 0;
      0 5 0 0;
      0 0 5 0;
      0 0 0 5];

% compute Q w/ van Loan's method
A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
Gamma = [0 0;0 0;1 0; 0 1];
w = 1; %spectral noise density
Z = dt * [-A Gamma*w*Gamma'; zeros(4) A'];
soln = expm(Z);
Q = soln(5:8,5:8)'*soln(1:4,5:8);

S = chol(Q,'lower');

for i=2:length(t)
%     pos_noise = 1*randn(2,1);
%     vel_noise = 0.01*randn(2,1);
    noise = S*randn(4,1);
    xtrue(:,i) = Phi * xtrue(:,i-1) + noise;% [pos_noise(1);pos_noise(2);vel_noise(1);vel_noise(2)];
end
  
% Q = [1 0.001 0 0;
%     0.001 1 0 0;
%     0 0 0.05 1E-4;
%     0 0 1E-4 0.05];

H1 = [1 0 0 0;
        0 1 0 0];
% H2 = [0 1 0 0];
%     0 1];
H1event = [1 0 0 0];
H2event = [0 1 0 0];
R = [5 0;0 5];% 0; 0 5];
Revent = 2;% 0;0 2];

% create state vector
x = zeros(4,length(t));
x(:,1) = x0;
P(:,:,1) = P0;

xdyn(:,1) = x0;
Pdyn(:,:,1) = P0;
xnormmeas(:,1) = x0;
Pnormmeas(:,:,1) = P0;
x_explicit(:,1) = x0;
P_explicit(:,:,1) = P0;

full_KF_msg_cnt = 0;
impl_msg_cnt1 = 0;
impl_msg_cnt2 = 0;

% update loop
for i=2:length(t)
    % propagate
    
    % full estimate
    x(:,i) = Phi * x(:,i-1); % state
    P(:,:,i) = Phi * P(:,:,i-1) * Phi' + Q;
    
    x_full_dyn(:) = x(:,i);
    
    % dynamics only
    xdyn(:,i) = Phi * xdyn(:,i-1);
    Pdyn(:,:,i) = Phi * Pdyn(:,:,i-1) * Phi' + Q;
    
    % platform measurements
    xnormmeas(:,i) = Phi * xnormmeas(:,i-1);
    Pnormmeas(:,:,i) = Phi * Pnormmeas(:,:,i-1) * Phi' + Q;
    
    % explicit measurements only
    x_explicit(:,i) = Phi * x_explicit(:,i-1);
    P_explicit(:,:,i) = Phi * P_explicit(:,:,i-1) * Phi';
    
    % measurement update
    % get measurement - platform sensors
%     for j=1:num_robots
%         z_plat(i) = xtrue(2*i-1,i) + 3*randn(1,1);
%         zevent(i) = xtrue(2*i-1,i) + 1*randn(1,1);
%     end
    
    z1 = xtrue(1:2,i) + 3*randn(2,1);
    z1event = xtrue(1,i) + 1*rand(1,1);
    z2 = xtrue(2,i) + 3*randn(1,1);
    z2event = xtrue(2,i) + 1*rand(1,1);

    % compute kalman gain
    
    % full estimate 
    
    K = P(:,:,i) * H1' * inv(H1 * P(:,:,i) * H1' + R);
%     Kmeas(:,i) = K;
    x(:,i) = x(:,i) + K*(z1-H1*x(:,i));
    P(:,:,i) = (eye(size(x(:,1)))-K*H1)*P(:,:,i)*(eye(size(x(:,1)))-K*H1)' + K*R*K';
    
%     K = P(:,:,i) * H2' * inv(H2 * P(:,:,i) * H2' + R);
%     Kmeas(:,i) = K;
%     x(:,i) = x(:,i) + K*(z2-H2*x(:,i));
%     P(:,:,i) = (eye(size(x(:,1)))-K*H2)*P(:,:,i)*(eye(size(x(:,1)))-K*H2)' + K*R*K';
    
    % platform estimate
    Knormeas = Pnormmeas(:,:,i) * H1' * inv(H1 * Pnormmeas(:,:,i) * H1' + R);
%     Kfull(:,i) = Knormeas;
    xnormmeas(:,i) = xnormmeas(:,i) + Knormeas*(z1-H1*xnormmeas(:,i));
    Pnormmeas(:,:,i) = (eye(size(xnormmeas(:,1)))-Knormeas*H1)*Pnormmeas(:,:,i)...
            *(eye(size(xnormmeas(:,1)))-Knormeas*H1)' + Knormeas*R*Knormeas';
        
    Knormeas = Pnormmeas(:,:,i) * H1event' * inv(H1event * Pnormmeas(:,:,i) * H1event' + Revent);
    xnormmeas(:,i) = xnormmeas(:,i) + Knormeas*(z1event-H1event*xnormmeas(:,i));
    Pnormmeas(:,:,i) = (eye(size(xnormmeas(:,1)))-Knormeas*H1event)*Pnormmeas(:,:,i)...
            *(eye(size(xnormmeas(:,1)))-Knormeas*H1event)' + Knormeas*Revent*Knormeas';
        
%     Knormeas = Pnormmeas(:,:,i) * H2' * inv(H2 * Pnormmeas(:,:,i) * H2' + R);
%     Kfull(:,i) = Knormeas;
%     xnormmeas(:,i) = xnormmeas(:,i) + Knormeas*(z2-H2*xnormmeas(:,i));
%     Pnormmeas(:,:,i) = (eye(size(xnormmeas(:,1)))-Knormeas*H2)*Pnormmeas(:,:,i)...
%             *(eye(size(xnormmeas(:,1)))-Knormeas*H2)' + Knormeas*R*Knormeas';
%         
%     Knormeas = Pnormmeas(:,:,i) * H2event' * inv(H2event * Pnormmeas(:,:,i) * H2event' + Revent);
%     xnormmeas(:,i) = xnormmeas(:,i) + Knormeas*(z2event-H2event*xnormmeas(:,i));
%     Pnormmeas(:,:,i) = (eye(size(xnormmeas(:,1)))-Knormeas*H2event)*Pnormmeas(:,:,i)...
%             *(eye(size(xnormmeas(:,1)))-Knormeas*H2event)' + Knormeas*Revent*Knormeas';
        
    full_KF_msg_cnt = full_KF_msg_cnt + 1;
    
    % platform measurement for explicit only estimate
%     Kexplicit = P_explicit(:,:,i) * H' * inv(H * P_explicit(:,:,i) * H' + R);
%     x_explicit(:,i) = x_explicit(:,i) + Kexplicit*(z-H*x_explicit(:,i));
%     P_explicit(:,:,i) = (eye(size(x_explicit(:,1)))-Kexplicit*H)*P_explicit(:,:,i)*(eye(size(x_explicit(:,1)))-Kexplicit*H)' + Kexplicit*R*Kexplicit';
    
    % event triggered measurement - explicit measurement
    innov(:,i) = z1event-H1event*x(:,i);
    if abs(z1event-H1event*x(:,i)) > delta
%         disp(zevent-Hevent*x(:,i))
        % full
        Kevent = P(:,:,i) * H1event' * inv(H1event * P(:,:,i) * H1event' + Revent);
        x(:,i) = x(:,i) + Kevent*(z1event-H1event*x(:,i));
        P(:,:,i) = (eye(size(x(:,1)))-Kevent*H1event)*P(:,:,i)*(eye(size(x(:,1)))-Kevent*H1event)' + Kevent*Revent*Kevent';
        eventvec(i,1) = t(i);

        impl_msg_cnt1 = impl_msg_cnt1 + 1;

    else % implicit measurement
        for j=1:length(z1event)
    %         mu = h(xhat(k)) - h(xbar(k))
%             mu = zevent;
            mu = H1event*x(:,i) - H1event*x_full_dyn(:);
            Qe = H1event*P(:,:,i)*H1event' + Revent;
            % a = h(x_ref) - h(xbar(k))
            a = z1event-H1event*x(:,i);
            
            arg1 = -delta+a-mu/sqrt(Qe);
            arg2 = delta+a-mu/sqrt(Qe);
            
%             zbar = (phi(-delta+a-mu/sqrt(Qe))-phi(delta+a-mu/sqrt(Qe)))/(Qfxn(-delta+a-mu/sqrt(Qe))-Qfxn(delta+a-mu/sqrt(Qe))) * sqrt(Qe);
%             dcal = (phi(-delta+a-mu/sqrt(Qe))-phi(delta+a-mu/sqrt(Qe)))/(Qfxn(-delta+a-mu/sqrt(Qe))-Qfxn(delta+a-mu/sqrt(Qe)))^2 ...
%                     - ((-delta+a-mu/sqrt(Qe))*phi(-delta+a-mu/sqrt(Qe))-(delta+a-mu/sqrt(Qe))*phi(delta+a-mu/sqrt(Qe)))/...
%                         (Qfxn(-delta+a-mu/sqrt(Qe))-Qfxn(delta+a-mu/sqrt(Qe)));
                    
            zbar = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2)))*sqrt(Qe);
            dcal = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2))^2) - ...
                        ((arg1)*phi(arg1)-arg2*phi(arg2)/(Qfxn(arg1)-Qfxn(arg2)));
                    
            Kevent = P(:,:,i) * H1event'*inv(H1event*P(:,:,i)*H1event' + Revent);
            Kimplicit(:,i) = Kevent;
            x(:,i) = x(:,i) + Kevent*zbar;
            invquant = inv(H1event*P(:,:,i)*H1event' + Revent);
            P(:,:,i) = P(:,:,i) - dcal*P(:,:,i)*H1event'*invquant*H1event*P(:,:,i);
%             P(:,:,i) = P(:,:,i)-dcal*K*P(:,:,i);
%             P(:,:,i) = (eye(2)-dcal*Kevent*Hevent)*Hevent*P(:,:,i);
            
%             mu = H2event*x(:,i) - H2event*x_full_dyn(:);
%             Qe = H2event*P(:,:,i)*H2event' + Revent;
%             % a = h(x_ref) - h(xbar(k))
%             a = z2event-H2event*x(:,i);
%             
%             arg1 = -delta+a-mu/sqrt(Qe);
%             arg2 = delta+a-mu/sqrt(Qe);
%             
% %             zbar = (phi(-delta+a-mu/sqrt(Qe))-phi(delta+a-mu/sqrt(Qe)))/(Qfxn(-delta+a-mu/sqrt(Qe))-Qfxn(delta+a-mu/sqrt(Qe))) * sqrt(Qe);
% %             dcal = (phi(-delta+a-mu/sqrt(Qe))-phi(delta+a-mu/sqrt(Qe)))/(Qfxn(-delta+a-mu/sqrt(Qe))-Qfxn(delta+a-mu/sqrt(Qe)))^2 ...
% %                     - ((-delta+a-mu/sqrt(Qe))*phi(-delta+a-mu/sqrt(Qe))-(delta+a-mu/sqrt(Qe))*phi(delta+a-mu/sqrt(Qe)))/...
% %                         (Qfxn(-delta+a-mu/sqrt(Qe))-Qfxn(delta+a-mu/sqrt(Qe)));
%                     
%             zbar = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2)))*sqrt(Qe);
%             dcal = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2))^2) - ...
%                         ((arg1)*phi(arg1)-arg2*phi(arg2)/(Qfxn(arg1)-Qfxn(arg2)));
%                     
%             Kevent = P(:,:,i) * H2event'*inv(H2event*P(:,:,i)*H2event' + Revent);
%             Kimplicit(:,i) = Kevent;
%             x(:,i) = x(:,i) + Kevent*zbar;
%             invquant = inv(H2event*P(:,:,i)*H2event' + Revent);
%             P(:,:,i) = P(:,:,i) - dcal*P(:,:,i)*H2event'*invquant*H2event*P(:,:,i);

        end
    end
    
    if abs(z2event-H2event*x(:,i)) > delta
%         disp(zevent-Hevent*x(:,i))
        % full
        Kevent = P(:,:,i) * H2event' * inv(H2event * P(:,:,i) * H2event' + Revent);
        x(:,i) = x(:,i) + Kevent*(z2event-H2event*x(:,i));
        P(:,:,i) = (eye(size(x(:,1)))-Kevent*H2event)*P(:,:,i)*(eye(size(x(:,1)))-Kevent*H2event)' + Kevent*Revent*Kevent';
        eventvec(i,1) = t(i);

        impl_msg_cnt2 = impl_msg_cnt2 + 1;
        
    else % implicit measurement
        for j=1:length(z1event)
    %         mu = h(xhat(k)) - h(xbar(k))
%             mu = zevent;
            
            mu = H2event*x(:,i) - H2event*x_full_dyn(:);
            Qe = H2event*P(:,:,i)*H2event' + Revent;
            % a = h(x_ref) - h(xbar(k))
            a = z2event-H2event*x(:,i);
            
            arg1 = -delta+a-mu/sqrt(Qe);
            arg2 = delta+a-mu/sqrt(Qe);
            
%             zbar = (phi(-delta+a-mu/sqrt(Qe))-phi(delta+a-mu/sqrt(Qe)))/(Qfxn(-delta+a-mu/sqrt(Qe))-Qfxn(delta+a-mu/sqrt(Qe))) * sqrt(Qe);
%             dcal = (phi(-delta+a-mu/sqrt(Qe))-phi(delta+a-mu/sqrt(Qe)))/(Qfxn(-delta+a-mu/sqrt(Qe))-Qfxn(delta+a-mu/sqrt(Qe)))^2 ...
%                     - ((-delta+a-mu/sqrt(Qe))*phi(-delta+a-mu/sqrt(Qe))-(delta+a-mu/sqrt(Qe))*phi(delta+a-mu/sqrt(Qe)))/...
%                         (Qfxn(-delta+a-mu/sqrt(Qe))-Qfxn(delta+a-mu/sqrt(Qe)));
                    
            zbar = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2)))*sqrt(Qe);
            dcal = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2))^2) - ...
                        ((arg1)*phi(arg1)-arg2*phi(arg2)/(Qfxn(arg1)-Qfxn(arg2)));
                    
            Kevent = P(:,:,i) * H2event'*inv(H2event*P(:,:,i)*H2event' + Revent);
            Kimplicit(:,i) = Kevent;
            x(:,i) = x(:,i) + Kevent*zbar;
            invquant = inv(H2event*P(:,:,i)*H2event' + Revent);
            P(:,:,i) = P(:,:,i) - dcal*P(:,:,i)*H2event'*invquant*H2event*P(:,:,i);

        end
    end
    
%     if abs(zevent-Hevent*x_explicit(:,i)) > delta
%         
%         Kevent_exp = P_explicit(:,:,i) * Hevent' * inv(Hevent * P_explicit(:,:,i) * Hevent' + Revent);
%         x_explicit(:,i) = x_explicit(:,i) + Kevent_exp*(zevent-Hevent*x(:,i));
%         P_explicit(:,:,i) = (eye(size(x_explicit(:,1)))-Kevent_exp*Hevent)*P_explicit(:,:,i)...
%             *(eye(size(x_explicit(:,1)))-Kevent_exp*Hevent)' + Kevent_exp*Revent*Kevent_exp';
%     end
        
        

    
end

fprintf('Full Kalman Filter got %i measurements from range sensor.\n',full_KF_msg_cnt);
fprintf('Implicit filter got %i measurements from range sensor.\n',impl_msg_cnt1);

% compute rms error of full FK and implicit filter
% position
for i=1:length(xtrue(1,:))
    full_KF_rms_pos(i) = sqrt((1/length(xnormmeas(1,1:i)))*sum((xnormmeas(1,1:i)-xtrue(1,1:i)).^2));
    impl_rms_pos(i) = sqrt((1/length(x(1,1:i)))*sum((x(1,1:i)-xtrue(1,1:i)).^2));
end

if plots

    errbars(:) = sqrt(P(1,1,:));

%     figure
%     subplot(3,2,1)
%     hold on; grid on;
%     plot(t,xtrue(1,:))
%     plot(t,xdyn(1,:))
%     xlabel('time')
%     ylabel('position')
%     title('dynamics propagation only')
%     legend('true position','dymaics prop')
%     
    subplot(2,1,1)
    hold on; grid on;
    plot(t,xtrue(1,:))
    plot(t,xnormmeas(1,:))
    plot(t,x(1,:))
    xlabel('time')
    ylabel('position [m]')
    title('filter peformance')
    legend('true position','full KF','implicit measurement filter')
    
%     subplot(2,2,3)
%     hold on; grid on;
%     plot(t,xtrue(1,:))
%     plot(t,x_explicit(1,:))
%     xlabel('time')
%     ylabel('position')
%     title('explicit updates from range')
%     legend('true position','explicit updates')
    
%     subplot(2,1,2)
%     hold on; grid on;
%     plot(t,xtrue(1,:))
%     plot(t,x(1,:))
% %     plot(t,x(1,:)+errbars,'k--')
% %     plot(t,x(1,:)-errbars,'k--')
%     xlabel('time')
%     ylabel('position')
%     title('implicit updates')
%     legend('true position','implicit est')

    subplot(2,1,2)
    hold on; grid on;
    plot(t,abs(xtrue(1,:)-xnormmeas(1,:)))
    plot(t,abs(xtrue(1,:)-x(1,:)))
    xlabel('time [s]')
    ylabel('error [m]')
    legend('KF error','implicit error')
   

%     h1 = figure;
%     hold on; grid on;
%     plot(t,xtrue(1,:))
%     plot(t,x(1,:))
%     plot(t,xdyn(1,:))
%     plot(t,xnormmeas(1,:))
%     plot(t,x_explicit(1,:))
% %     plot(t,x(1,:) + errbars,'--r')
% %     plot(t,x(1,:) - errbars,'--r')
%     legend('true pos','pos est','\pm 1\sigma')
%     xlabel('Time [s]')
%     ylabel('Position [m]')
%     title('Estimated and true position over time with error bounds')
% 
    h2 = figure;
    hold on; grid on;
    plot(t,abs(x(1,:)-xtrue(1,:)))
    plot(eventvec(:),zeros(length(eventvec),1),'or')
    legend('pos error','explicit event update')
    xlabel('Time [s]')
    ylabel('Error [m]')
    title('Filter position error with explicit updates')
% 
%     h3 = figure;
%     hold on; grid on;
%     plot(t,xtrue(1,:)-x(1,:))
%     plot(t,2*errbars)
%     plot(t,-2*errbars)
% rank
%     errvelbars(:) = sqrt(P(2,2,:));
%     h4 = figure;
%     hold on; grid on;
%     plot(t,xtrue(2,:)-x(2,:))
%     plot(t,2*errvelbars)
%     plot(t,-2*errvelbars)

    figure
    hold on; grid on;
    plot(t,full_KF_rms_pos)
    plot(t,impl_rms_pos)
    xlabel('Time [s]')
    ylabel('RMS error [m]')
    title('RMS Position Error for Full and Implicit KF')
    full_KF_led_str = strcat('full KF: ',int2str(full_KF_msg_cnt),' messages sent');
    impl_led_str = strcat('implicit KF: ',int2str(impl_msg_cnt1),' messages sent');
    legend(full_KF_led_str,impl_led_str,'Location','SouthEast')
    
    figure
    
    subplot(2,1,1)
    hold on; grid on;
    plot(t,innov)
    plot(t,delta*ones(length(t),1),'--b')
    plot(t,-delta*ones(length(t),1),'--b')
    xlabel('Time [s]')
    ylabel('Innovation [m]')
    legend('innovation','\pm \delta')
    
%     subplot(2,1,2)
%     hold on; grid on;
%     plot(t,Kfull(1,:))
%     plot(t,Kmeas(1,:))
%     plot(t,Kimplicit(1,:))
%     xlabel('Time [s]')
%     ylabel('Kalman gain')
%     legend('full KF','Implictit KF meas','implicit update')
    
    figure
    hold on; grid on;
    plot(xtrue(1,:),xtrue(2,:))
    plot(xnormmeas(1,:),xnormmeas(2,:))
    plot(x(1,:),x(2,:))
    
    
%     x_errbars(:) = sqrt(P(1,1,:));
%     y_errbars(:) = sqrt(P(2,2,:));
%     xvel_errbars(:) = sqrt(P(3,3,:));
%     yvel_errbars(:) = sqrt(P(4,4,:));
    
    x_errbars(:) = sqrt(Pnormmeas(1,1,:));
    y_errbars(:) = sqrt(Pnormmeas(2,2,:));
    xvel_errbars(:) = sqrt(Pnormmeas(3,3,:));
    yvel_errbars(:) = sqrt(Pnormmeas(4,4,:));
    
    figure
    subplot(2,2,1)
    hold on; grid on;
    plot(t,xtrue(1,:)-xnormmeas(1,:))
    plot(t,xtrue(1,:)-x(1,:))
    plot(t,xtrue(1,:)-x(1,:) + 2.*x_errbars,'-.r')
    plot(t,xtrue(1,:)-x(1,:) - 2.*x_errbars,'-.r')
    xlabel('t')
    ylabel('error')
    title('x pos error')
    
    subplot(2,2,2)
    hold on; grid on;
    plot(t,xtrue(2,:)-xnormmeas(2,:))
    plot(t,xtrue(2,:)-x(2,:))
    plot(t,xtrue(2,:)-x(2,:) + 2.*y_errbars,'-.r')
    plot(t,xtrue(2,:)-x(2,:) - 2.*y_errbars,'-.r')
    xlabel('t')
    ylabel('error')
    title('y pos error')
    
    subplot(2,2,3)
    hold on; grid on;
    plot(t,xtrue(3,:)-xnormmeas(3,:))
    plot(t,xtrue(3,:)-x(3,:))
    plot(t,xtrue(3,:)-x(3,:) + 2.*xvel_errbars,'-.r')
    plot(t,xtrue(3,:)-x(3,:) - 2.*xvel_errbars,'-.r')
    xlabel('t')
    ylabel('error')
    title('x vel error')
    
    subplot(2,2,4)
    hold on; grid on;
    plot(t,xtrue(4,:)-xnormmeas(4,:))
    plot(t,xtrue(4,:)-x(4,:))
    plot(t,xtrue(4,:)-x(4,:) + 2.*yvel_errbars,'-.r')
    plot(t,xtrue(4,:)-x(4,:) - 2.*yvel_errbars,'-.r')
    xlabel('t')
    ylabel('error')
    title('y vel error')
    

end





% create animation
if animation
    h5 = figure;
    cla;
    gcf;
    hold on;
    axis tight manual
    fn = 'testgif.gif';
    for i=1:length(t)
        mu = x(1,i);
        sigma = sqrt(P(1,1,i));
        plotx = mu-3*sigma:0.01:mu+3*sigma;
        plot(plotx,normpdf(plotx,mu,sigma))

        mudyn = xdyn(1,i);
        sigma_dyn = sqrt(Pdyn(1,1,i));
        plotx_dyn = mudyn-3*sigma_dyn:0.01:mudyn+3*sigma_dyn;
        plot(plotx_dyn,normpdf(plotx_dyn,mudyn,sigma_dyn));

        mu_norm = xnormmeas(1,i);
        sigma_norm = sqrt(Pnormmeas(1,1,i));
        plotx_norm = mu_norm-3*sigma_norm:0.01:mu_norm+3*sigma_norm;
        plot(plotx_norm,normpdf(plotx_norm,mu_norm,sigma_norm))
        
        mu_exp = x_explicit(1,i);
        sigma_exp = sqrt(P_explicit(1,1,i));
        plotx_exp = mu_exp-3*sigma_exp:0.01:mu_exp+3*sigma_exp;
        plot(plotx_exp,normpdf(plotx_exp,mu_exp,sigma_exp))

        line([xtrue(1,i) xtrue(1,i)], [0 0.5])

        legend('exp & imp. updates','dynamics only','norm meas. update','explicit')

        axis([-20 20 0 0.5]);
    %     drawnow

        frame = getframe(h5);
    %     M(i) = frame;
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if i==1
            imwrite(imind,cm,fn,'gif','Loopcount',inf);
        else
            imwrite(imind,cm,fn,'gif','WriteMode','append');
        end
        cla;
    end
end
    