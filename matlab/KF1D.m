% 1D Kalman filter implementation
% 
% Robot can travel on a line, can take measurements, and also receive
% meaurements from a ranging device at the origin (event-triggering bit,
% not yet implemented)

clear; close all; clc;

% state vector
% x = [x xdot]
rng(965);
animation = false;
plots = true;

% timespan
tspan = [0 50]; %[s]
% time vector
dt = 0.1;
t = tspan(1):dt:tspan(2);


% event triggering threshold
delta = 1;
% implicit measurement pdf fxn handle
phi = @(z) (1/sqrt(2*pi)) * exp(-0.5*z^2);
% Qfxn = @(z) 0.5*erfc(-z/sqrt(2));
Qfxn = @(z) normcdf(z,0,1,'upper');

% dynamics

% STM
Phi = [1 dt;
        0 1];
    
%% true trajectory
% initial conditions
x0 = [0; 0.05];
xtrue = zeros(2,length(t));
xtrue(:,1) = x0;
    
for i=2:length(t)
    pos_noise = 1*randn(1,1);
    vel_noise = 0.2*randn(1,1);
    xtrue(:,i) = Phi * xtrue(:,i-1) + [pos_noise;vel_noise];
end

%% KF

% initial conditions
x0 = [0 0.1];
P0 = [5 0; 0 5];
Q = [2 0; 0 2];
H = [1 0];
%     0 1];
Hevent = [1 0];
R = 5;% 0; 0 5];
Revent = 2;

% create state vector
x = zeros(2,length(t));
x(:,1) = x0;
P(:,:,1) = P0;

xdyn(:,1) = x0;
Pdyn(:,:,1) = P0;
xnormmeas(:,1) = x0;
Pnormmeas(:,:,1) = P0;
x_explicit(:,1) = x0;
P_explicit(:,:,1) = P0;

full_KF_msg_cnt = 0;
impl_msg_cnt = 0;

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
    z = xtrue(1,i) + sqrt(R)*randn(1,1);
    zevent = xtrue(1,i) + 1*rand(1,1);
    % compute kalman gain
    
    % full estimate
    K = P(:,:,i) * H' * inv(H * P(:,:,i) * H' + R);
    Kmeas(:,i) = K;
    x(:,i) = x(:,i) + K*(z-H*x(:,i));
    P(:,:,i) = (eye(size(x(:,1)))-K*H)*P(:,:,i)*(eye(size(x(:,1)))-K*H)' + K*R*K';
%     P(:,:,i) = (eye(size(x(:,1)))-K*H)*P(:,:,i);
    
    % platform estimate
    Knormeas = Pnormmeas(:,:,i) * H' * inv(H * Pnormmeas(:,:,i) * H' + R);
    Kfull(:,i) = Knormeas;
    xnormmeas(:,i) = xnormmeas(:,i) + Knormeas*(z-H*xnormmeas(:,i));
    Pnormmeas(:,:,i) = (eye(size(xnormmeas(:,1)))-Knormeas*H)*Pnormmeas(:,:,i)...
            *(eye(size(xnormmeas(:,1)))-Knormeas*H)' + Knormeas*R*Knormeas';
        
    Knormeas = Pnormmeas(:,:,i) * Hevent' * inv(Hevent * Pnormmeas(:,:,i) * Hevent' + Revent);
    xnormmeas(:,i) = xnormmeas(:,i) + Knormeas*(zevent-Hevent*xnormmeas(:,i));
    Pnormmeas(:,:,i) = (eye(size(xnormmeas(:,1)))-Knormeas*Hevent)*Pnormmeas(:,:,i)...
            *(eye(size(xnormmeas(:,1)))-Knormeas*Hevent)' + Knormeas*Revent*Knormeas';
        
    full_KF_msg_cnt = full_KF_msg_cnt + 1;
    
    % platform measurement for explicit only estimate
    Kexplicit = P_explicit(:,:,i) * H' * inv(H * P_explicit(:,:,i) * H' + R);
    x_explicit(:,i) = x_explicit(:,i) + Kexplicit*(z-H*x_explicit(:,i));
    P_explicit(:,:,i) = (eye(size(x_explicit(:,1)))-Kexplicit*H)*P_explicit(:,:,i)*(eye(size(x_explicit(:,1)))-Kexplicit*H)' + Kexplicit*R*Kexplicit';
    
    % event triggered measurement - explicit measurement
    innov(i) = zevent-Hevent*x(:,i);
    if abs(zevent-Hevent*x(:,i)) > delta
%         disp(zevent-Hevent*x(:,i))
        % full
        Kevent = P(:,:,i) * Hevent' * inv(Hevent * P(:,:,i) * Hevent' + Revent);
        x(:,i) = x(:,i) + Kevent*(zevent-Hevent*x(:,i));
        P(:,:,i) = (eye(size(x(:,1)))-Kevent*Hevent)*P(:,:,i)*(eye(size(x(:,1)))-Kevent*Hevent)' + Kevent*Revent*Kevent';
        eventvec(i,1) = t(i);

        impl_msg_cnt = impl_msg_cnt + 1;

    else % implicit measurement
        for j=1:length(zevent)
    %         mu = h(xhat(k)) - h(xbar(k))
%             mu = zevent;
            mu = Hevent*x(:,i) - Hevent*x_full_dyn(:);
            Qe = Hevent*P(:,:,i)*Hevent' + Revent;
            % a = h(x_ref) - h(xbar(k))
            a = zevent-Hevent*x(:,i);
            
            arg1 = -delta+a-mu/sqrt(Qe);
            arg2 = delta+a-mu/sqrt(Qe);
            
%             zbar = (phi(-delta+a-mu/sqrt(Qe))-phi(delta+a-mu/sqrt(Qe)))/(Qfxn(-delta+a-mu/sqrt(Qe))-Qfxn(delta+a-mu/sqrt(Qe))) * sqrt(Qe);
%             dcal = (phi(-delta+a-mu/sqrt(Qe))-phi(delta+a-mu/sqrt(Qe)))/(Qfxn(-delta+a-mu/sqrt(Qe))-Qfxn(delta+a-mu/sqrt(Qe)))^2 ...
%                     - ((-delta+a-mu/sqrt(Qe))*phi(-delta+a-mu/sqrt(Qe))-(delta+a-mu/sqrt(Qe))*phi(delta+a-mu/sqrt(Qe)))/...
%                         (Qfxn(-delta+a-mu/sqrt(Qe))-Qfxn(delta+a-mu/sqrt(Qe)));
                    
            zbar = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2)))*sqrt(Qe);
            dcal = ((phi(arg1)-phi(arg2))/(Qfxn(arg1)-Qfxn(arg2))^2) - ...
                        ((arg1)*phi(arg1)-arg2*phi(arg2)/(Qfxn(arg1)-Qfxn(arg2)));
                    
            Kevent = P(:,:,i) * Hevent'*inv(Hevent*P(:,:,i)*Hevent' + Revent);
            Kimplicit(:,i) = Kevent;
            x(:,i) = x(:,i) + Kevent*zbar;
            invquant = inv(Hevent*P(:,:,i)*Hevent' + Revent);
            P(:,:,i) = P(:,:,i) - dcal*P(:,:,i)*Hevent'*invquant*Hevent*P(:,:,i);
%             P(:,:,i) = P(:,:,i)-dcal*K*P(:,:,i);
%             P(:,:,i) = (eye(2)-dcal*Kevent*Hevent)*Hevent*P(:,:,i);
            
        end
    end
    
    if abs(zevent-Hevent*x_explicit(:,i)) > delta
        
        Kevent_exp = P_explicit(:,:,i) * Hevent' * inv(Hevent * P_explicit(:,:,i) * Hevent' + Revent);
        x_explicit(:,i) = x_explicit(:,i) + Kevent_exp*(zevent-Hevent*x(:,i));
        P_explicit(:,:,i) = (eye(size(x_explicit(:,1)))-Kevent_exp*Hevent)*P_explicit(:,:,i)...
            *(eye(size(x_explicit(:,1)))-Kevent_exp*Hevent)' + Kevent_exp*Revent*Kevent_exp';
    end
        
        

    
end

fprintf('Full Kalman Filter got %i measurements from range sensor.\n',full_KF_msg_cnt);
fprintf('Implicit filter got %i measurements from range sensor.\n',impl_msg_cnt);

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
    full_KF_led_str = strcat('full KF: ',int2str(full_KF_msg_cnt),' measurements sent');
    impl_led_str = strcat('implicit KF: ',int2str(impl_msg_cnt),' measurements sent');
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
    
    subplot(2,1,2)
    hold on; grid on;
    plot(t,Kfull(1,:))
    plot(t,Kmeas(1,:))
    plot(t,Kimplicit(1,:))
    xlabel('Time [s]')
    ylabel('Kalman gain')
    legend('full KF','Implictit KF meas','implicit update')

    

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
    