% FUSION paper mse plots

% assumes network_mse matrix in workspace

color_wheel = [0    0.4470    0.7410;
    0.8500    0.3250    0.0980;
    0.9290    0.6940    0.1250;
    0.4940    0.1840    0.5560;
    0.4660    0.6740    0.1880;
    0.3010    0.7450    0.9330;
    0.6350    0.0780    0.1840];

% plot general info, indep of network config
% figure
% subplot(3,1,1)
% hold on; grid on;
% for i=1:size(avg_mse,1)
%     plot(input_tvec,avg_mse(i,:))
% end
% title('Avg per agent position MSE')
% xlabel('Time [s]')
% ylabel('MSE [m^2]')
% 
% subplot(3,1,2)
% hold on; grid on;
% for i=1:size(avg_xmse,1)
%     plot(input_tvec,avg_xmse(i,:))
% end
% title('Avg per agent x position MSE')
% xlabel('Time [s]')
% ylabel('MSE [m^2]')
% 
% subplot(3,1,3)
% hold on; grid on;
% for i=1:size(avg_ymse,1)
%     plot(input_tvec,avg_ymse(i,:))
% end
% title('Avg per agent y position MSE')
% xlabel('Time [s]')
% ylabel('MSE [m^2]')

gps_id = 18;
mid_id = 22;
far_id = 30;

rel_gps = 16;
rel_mid = 18;
rel_far = 22;

loop_var = tau_state_goal_vec;

skip = 10;

figure
hold on; grid on;
for i=1:length(loop_var)
    plot(input_tvec(1:skip:end),network_mse(gps_id,1:skip:end,i),'-d','Color',color_wheel(i+3,:))
end
plot(input_tvec(1:skip:end),baseline_mse(gps_id,1:skip:end,1),'-ko')
title('Agent 18 position MSE')
xlabel('Time [s]')
ylabel('MSE [m^2]')
legend('\delta=0.5','\delta=1','\delta=1.5','\delta=2','cent KF')

figure
hold on; grid on;
for i=1:length(loop_var)
    plot(input_tvec(1:skip:end),network_mse(mid_id,1:skip:end,i),'-d')
end
plot(input_tvec(1:skip:end),baseline_mse(mid_id,1:skip:end,1),'-ko')
title('Agent 22 position MSE')
xlabel('Time [s]')
ylabel('MSE [m^2]')
legend('\delta=0.5','\delta=1','\delta=1.5','\delta=2','cent KF')

figure
hold on; grid on;
for i=1:length(loop_var)
    plot(input_tvec(1:skip:end),network_mse(far_id,1:skip:end,i),'-d')
end
plot(input_tvec(1:skip:end),baseline_mse(far_id,1:skip:end,1),'-ko')
title('Agent 30 position MSE')
xlabel('Time [s]')
ylabel('MSE [m^2]')
legend('\delta=0.5','\delta=1','\delta=1.5','\delta=2','cent KF')




figure
hold on; grid on;
for i=1:length(loop_var)
    plot(input_tvec(1:skip:end),squeeze(rel_network_mse(gps_id,rel_gps,1:skip:end,i)),'-d')
end
plot(input_tvec(1:skip:end),squeeze(rel_baseline_mse(gps_id,rel_gps,1:skip:end,1)),'-ko')
title('Agent 16-18 position MSE')
xlabel('Time [s]')
ylabel('MSE [m^2]')
legend('\delta=0.5','\delta=1','\delta=1.5','\delta=2','cent KF')

figure
hold on; grid on;
for i=1:length(loop_var)
    plot(input_tvec(1:skip:end),squeeze(rel_network_mse(mid_id,rel_mid,1:skip:end,i)),'-d')
end
plot(input_tvec(1:skip:end),squeeze(rel_baseline_mse(mid_id,rel_mid,1:skip:end,1)),'-ko')
title('Agent 18-22 position MSE')
xlabel('Time [s]')
ylabel('MSE [m^2]')
legend('\delta=0.5','\delta=1','\delta=1.5','\delta=2','cent KF')

figure
hold on; grid on;
for i=1:length(loop_var)
    plot(input_tvec(1:skip:end),squeeze(rel_network_mse(far_id,rel_far,1:skip:end,i)),'-d')
end
plot(input_tvec(1:skip:end),squeeze(rel_baseline_mse(far_id,rel_far,1:skip:end,1)),'-ko')
title('Agent 22-30 position MSE')
xlabel('Time [s]')
ylabel('MSE [m^2]')
legend('\delta=0.5','\delta=1','\delta=1.5','\delta=2','cent KF')

% figure
% subplot(2,1,1)
% heatmap(network_mse(1,:,1))
% title('Network position MSE')
% xlabel('Time')
% ylabel('Agent')
% 
% subplot(2,1,2)
% heatmap(network_mse(:,:,1))
% title('Network position MSE')
% xlabel('Time')
% ylabel('Agent')