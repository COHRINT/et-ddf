%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Normalized Estimation Error Sqaured (NEES) KF Consistency test
%
% Ian Loefgren
% 11.16.2018
%
% Given a linear KF, simulates N Monte Carlo simulations of truth and
% filter data and performs chi2 hypothesis test to determine if the NEES
% data have a chi2 distribution as expected.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [consistent] = nees(filter,alpha)

% number of Monte Carlo simulations
N = 100;

dt = 0.1;
% conduct simulations

% generate input
input_time = 0:dt:15;
u = 3*cos(0.75*input_time);

% run Monte Carlo sims
Rtrue = filter.R;
Qtrue = filter.Q;
x_t = mvnrnd(zeros(1,size(filter.F,1)),eye(size(filter.F)))';
for i=1:N
    filter.x = x_t;
    filter.P = eye(size(filter.F));
    for j = 1:length(input_time)
        % simulate process noise
        w = mvnrnd(zeros(1,size(Qtrue,1)),Qtrue)';
        x_true = filter.F*x_t + filter.G*[u(:,j);u(:,j)] + w;

        % simulate measurement noise
        v = mvnrnd(zeros(1,2),Rtrue)';
        y_meas = filter.H*x_true + v;

        x_true_vec{i}(:,j) = x_true;
        y_meas_vec{i}(:,j) = y_meas;
        x_t = x_true;
        
        % update filter
        [x_pred_curr,P_pred_curr] = filter.predict([u(:,j);u(:,j)]);
        x_filt_pred{i}(:,j) = x_pred_curr;
        P_filt_pred{i}(:,:,j) = P_pred_curr;
        [x_curr,P_curr] = filter.update(y_meas);
        x_filt_est{i}(:,j) = x_curr;
        P_filt_est{i}(:,:,j) = P_curr;
        
        % compute innovation, NEES and NIS
        S = filter.H*P_pred_curr*filter.H' + Rtrue;
        nis_samp(i,j) = (y_meas - filter.H*x_pred_curr)'*inv(S)*(y_meas - filter.H*x_pred_curr);
        nees_samp(i,j) = (x_t - x_curr)'*inv(P_curr)*(x_t - x_curr);
        
    end
end

nis_samps = mean(nis_samp,1);
nees_samps = mean(nees_samp,1);

r1 = chi2inv(alpha/2,N*size(filter.F,1))./N;
r2 = chi2inv(1-alpha/2,N*size(filter.F,1))./N;

r1y = chi2inv(alpha/2,N*size(filter.H,1))./N;
r2y = chi2inv(1-alpha/2,N*size(filter.H,1))./N;

% figure
% hold on; grid on;
% plot(input_time,r1y*ones(length(input_time),1),'--r');
% plot(input_time,r2y*ones(length(input_time),1),'--r');
% plot(input_time,nis_samps,'xb')

disp(sum(nees_samps < r1 | nees_samps > r2))
disp(length(input_time))
% 
disp(sum(nis_samps < r1y | nis_samps > r2y))
disp(length(input_time))

% if both nees and nis values at each time step are less than significance
% level, the filter passes consistency test
consistent = (alpha > sum(nees_samps < r1 | nees_samps > r2)/length(input_time))...
        & (alpha > sum(nis_samps < r1y | nis_samps > r2y)/length(input_time)); 

end