% FUSION paper mse plots

% assumes network_mse matrix in workspace

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

gps_id = 13;
mid_id = 10;
far_id = 4;

loop_var = delta_vec;

figure
hold on; grid on;
for i=1:length(loop_var)
    plot(input_tvec(1:10:end),network_mse(gps_id,1:10:end,i),'-d')
end
plot(input_tvec(1:10:end),baseline_mse(gps_id,1:10:end,1),'-ko')
title('Agent 13 position MSE')
xlabel('Time [s]')
ylabel('MSE [m^2]')
legend('\delta=0.5','\delta=1','\delta=1.5','\delta=2','cent KF')

figure
hold on; grid on;
for i=1:length(loop_var)
    plot(input_tvec(1:10:end),network_mse(mid_id,1:10:end,i),'-d')
end
plot(input_tvec(1:10:end),baseline_mse(mid_id,1:10:end,1),'-ko')
title('Agent 10 position MSE')
xlabel('Time [s]')
ylabel('MSE [m^2]')
legend('\delta=0.5','\delta=1','\delta=1.5','\delta=2','cent KF')

figure
hold on; grid on;
for i=1:length(loop_var)
    plot(input_tvec(1:10:end),network_mse(far_id,1:10:end,i),'-d')
end
plot(input_tvec(1:10:end),baseline_mse(far_id,1:10:end,1),'-ko')
title('Agent 4 position MSE')
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