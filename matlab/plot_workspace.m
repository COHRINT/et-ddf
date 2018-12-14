% plotting script

% close all

% figure
% hold on; grid on;
% 
% plot(agents{1}.true_state(1,:),agents{1}.true_state(3,:))
% plot(agents{N}.true_state(1,:),agents{N}.true_state(3,:))

%% compute statistics
chain = 1;
comms_plots = 0;

msg_sent = 0;
ci_triggered = 0;
ci_triggered_vec = zeros(1,length(agents));
total = 0;

for i=1:length(agents)
    msg_sent = msg_sent + agents{i}.msgs_sent;
    total = total + agents{i}.total_msgs;
    ci_triggered = ci_triggered + agents{i}.ci_trigger_cnt;
    ci_triggered_vec(i) = agents{i}.ci_trigger_cnt;
end

data_transfer = 4*ci_triggered* N + 2*msg_sent;

fprintf('\n--------------------------------------------\n')
fprintf('For delta = %.3f, tau = %.3f, N = %i\n',delta,tau,N)
fprintf('Total messages sent throughout system: %i/%i\n',msg_sent,total)
fprintf('Number of times CI performed: %i/%i\n',ci_triggered,max_time*N/dt)
fprintf('Msg units (floats) transmitted overall: %i\n',data_transfer)
fprintf('--------------------------------------------\n')


comm_mat = zeros(N);
comm_mat_frac = zeros(N);
total_msgs_sent = 0;
possible_msgs = 0;
for i=1:N
    for j=1:length(agents{i}.common_estimates)
        comm_mat(i,agents{i}.common_estimates{j}.connection) = agents{i}.common_estimates{j}.msg_sent;
%         if i==11 || i==18
%             adj_total_msg = 600;
%         else
%         if agents{i}.common_estimates{j}.total_msg == 600
%             adj_total_msg = 400;
%         else
%             adj_total_msg = agents{i}.common_estimates{j}.total_msg;
%         end
        comm_mat_frac(i,agents{i}.common_estimates{j}.connection) = agents{i}.common_estimates{j}.msg_sent/agents{i}.common_estimates{j}.total_msg;
        total_msgs_sent = total_msgs_sent + agents{i}.common_estimates{j}.msg_sent;
        possible_msgs = possible_msgs + agents{i}.common_estimates{j}.total_msg;
    end
end

disp(total_msgs_sent)
disp(possible_msgs)
disp(total_msgs_sent/possible_msgs)

if chain
comm_compact = zeros(2,N);
% ci and communication heatmaps
for i=1:N
    [~,cols,vals] = find(comm_mat(i,:))
    if i==1
        vals = [0;vals];
%     elseif i==30
%         vals = [vals;0];
    elseif i==N
        vals = [vals;0];
%     elseif isempty(vals)
%         vals = [0;0;0];
    end
    comm_compact(:,i) = vals;
end
figure
subplot(2,1,1)
h = heatmap(1:N,["n-1","n+1"],comm_compact);
h.GridVisible = 'off';
h.ColorLimits = [min(comm_compact(comm_compact>0)) max(max(comm_compact))];
% colormap(h,parula);
xlabel('Source agent')
ylabel('Destination agent')
title('Agent communication')
subplot(2,1,2)
h = heatmap(ci_triggered_vec);
h.GridVisible = 'off';
% colormap(h,parula);
xlabel('Agent')
title('CI trigger totals')

comm_frac_compact = zeros(2,N);
% ci and communication heatmaps
for i=1:N
    [~,cols,vals] = find(comm_mat_frac(i,:));
    if i==1
        vals = [0;vals];
%     elseif i==N
%         vals = [vals;0];
    elseif i==N
        vals = [vals;0];
%     elseif isempty(vals)
%         vals = [0;0;0];
    end
    comm_frac_compact(:,i) = vals;
end
figure
subplot(2,1,1)
h = heatmap(1:N,["n-1","n+1"],comm_frac_compact);
h.GridVisible = 'off';
h.ColorLimits = [min(comm_frac_compact(comm_frac_compact>0)) max(max(comm_frac_compact))];
% colormap(h,parula);
xlabel('Source agent')
ylabel('Destination agent')
title('Agent communication rate')
subplot(2,1,2)
h = heatmap(ci_triggered_vec./(max_time/dt));
h.GridVisible = 'off';
% colormap(h,parula);
xlabel('Agent')
title('CI trigger rates')

figure
subplot(2,1,1)
h = heatmap(1:N,1:N,comm_mat);
h.GridVisible = 'off';
h.ColorLimits = [floor(min(comm_mat(comm_mat>0))) ceil(max(max(comm_mat)))];
% colormap(h,parula);
xlabel('Destination agent')
ylabel('Source agent')
title('Agent communication')
subplot(2,1,2)
h = heatmap(ci_triggered_vec);
h.GridVisible = 'off';
% colormap(h,parula);
xlabel('Agent')
title('CI trigger totals')

figure
subplot(2,1,1)
h = heatmap(1:N,1:N,comm_mat_frac);
h.GridVisible = 'off';
h.ColorLimits = [min(comm_mat_frac(comm_mat_frac>0)) max(max(comm_mat_frac))];
% colormap(h,parula);
xlabel('Destination agent')
ylabel('Source agent')
title('Agent communication rate')
subplot(2,1,2)
h = heatmap(ci_triggered_vec./(max_time/dt));
h.GridVisible = 'off';
% colormap(h,parula);
xlabel('Agent')
title('CI trigger rates')

else
    comm_mat = zeros(N);
comm_mat_frac = zeros(N);
total_msgs_sent = 0;
possible_msgs = 0;
for i=1:N
    for j=1:length(agents{i}.common_estimates)
        comm_mat(i,agents{i}.common_estimates{j}.connection) = agents{i}.common_estimates{j}.msg_sent;
        comm_mat_frac(i,agents{i}.common_estimates{j}.connection) = agents{i}.common_estimates{j}.msg_sent/agents{i}.common_estimates{j}.total_msg;
        total_msgs_sent = total_msgs_sent + agents{i}.common_estimates{j}.msg_sent;
        possible_msgs = possible_msgs + agents{i}.common_estimates{j}.total_msg;
    end
end

disp(total_msgs_sent)
disp(possible_msgs)
disp(total_msgs_sent/possible_msgs)

comm_compact = zeros(3,N);
% ci and communication heatmaps
for i=1:N
    [~,cols,vals] = find(comm_mat(i,:));
    if i>16
        if length(vals)==1
            vals = [0;0;vals];
    %     elseif i==N
    %         vals = [vals;0];
        elseif length(vals)==2
            vals = [0;vals'];
        elseif isempty(vals)
            vals = [0;0;0];
        end
    else
        if length(vals)==1
            vals = [0;0;vals];
    %     elseif i==N
    %         vals = [vals;0];
        elseif length(vals)==2
            vals = [0;vals'];
        elseif length(vals)==3
            vals = [vals(2);vals(3);vals(1)];
        elseif isempty(vals)
            vals = [0;0;0];
        end
    end
    comm_compact(:,i) = vals;
end
figure
subplot(2,1,1)
h = heatmap(1:N,["Child 1","Child 2","Parent"],comm_compact);
h.GridVisible = 'off';
h.ColorLimits = [min(comm_compact(comm_compact>0)) max(max(comm_compact))];
% colormap(h,parula);
xlabel('Source agent')
ylabel('Destination agent')
title('Agent communication')
subplot(2,1,2)
h = heatmap(ci_triggered_vec);
h.GridVisible = 'off';
% colormap(h,parula);
xlabel('Agent')
title('CI trigger totals')

comm_frac_compact = zeros(3,N);
% ci and communication heatmaps
for i=1:N
    [~,cols,vals] = find(comm_mat_frac(i,:));
    if i>16
        if length(vals)==1
            vals = [0;0;vals];
    %     elseif i==N
    %         vals = [vals;0];
        elseif length(vals)==2
            vals = [0;vals'];
        elseif isempty(vals)
            vals = [0;0;0];
        end
    else
        if length(vals)==1
            vals = [0;0;vals];
    %     elseif i==N
    %         vals = [vals;0];
        elseif length(vals)==2
            vals = [0;vals'];
        elseif length(vals)==3
            vals = [vals(2);vals(3);vals(1)];
        elseif isempty(vals)
            vals = [0;0;0];
        end
    end
    comm_frac_compact(:,i) = vals;
end
figure
subplot(2,1,1)
h = heatmap(1:N,["Child 1","Child 2","Parent"],comm_frac_compact);
h.GridVisible = 'off';
h.ColorLimits = [min(comm_frac_compact(comm_frac_compact>0)) max(max(comm_frac_compact))];
% colormap(h,parula);
xlabel('Source agent')
ylabel('Destination agent')
title('Agent communication rate')
subplot(2,1,2)
h = heatmap(ci_triggered_vec./(max_time/dt));
h.GridVisible = 'off';
% colormap(h,parula);
xlabel('Agent')
title('CI trigger rates')

figure
subplot(2,1,1)
h = heatmap(1:N,1:N,comm_mat);
h.GridVisible = 'off';
h.ColorLimits = [floor(min(comm_mat(comm_mat>0))) ceil(max(max(comm_mat)))];
% colormap(h,parula);
xlabel('Destination agent')
ylabel('Source agent')
title('Agent communication')
subplot(2,1,2)
h = heatmap(ci_triggered_vec);
h.GridVisible = 'off';
% colormap(h,parula);
xlabel('Agent')
title('CI trigger totals')

figure
subplot(2,1,1)
h = heatmap(1:N,1:N,comm_mat_frac);
h.GridVisible = 'off';
h.ColorLimits = [min(comm_mat_frac(comm_mat_frac>0)) max(max(comm_mat_frac))];
% colormap(h,parula);
xlabel('Destination agent')
ylabel('Source agent')
title('Agent communication rate')
subplot(2,1,2)
h = heatmap(ci_triggered_vec./(max_time/dt));
h.GridVisible = 'off';
% colormap(h,parula);
xlabel('Agent')
title('CI trigger rates')
end


%% estimation error plots
num_agents = N;
sigma = 3;

agent_plots_1 = [18 16 10 13 13 13];
agent_plots_2 = [18 18 18 13 15 22];
% agent_plots_1 = [1 3];
% agent_plots_2 = [1 6];


color_wheel = [0    0.4470    0.7410;
    0.8500    0.3250    0.0980;
    0.9290    0.6940    0.1250;
    0.4940    0.1840    0.5560;
    0.4660    0.6740    0.1880;
    0.3010    0.7450    0.9330;
    0.6350    0.0780    0.1840];



figure
for i=1:length(agent_plots_1)
    
j = agent_plots_1(i);
k = agent_plots_2(i);

% if i>3
% figure
% end

subplot(2,3,i)
hold on; grid on;

set(gca,'FontSize',13)

if j~=k
    plot(input_tvec,agents{j}.local_filter.state_history(4*(k-1)+1,:) - agents{k}.true_state(1,:),'Color',color_wheel(1,:))
    plot(input_tvec,sigma*sqrt(squeeze(agents{j}.local_filter.cov_history(4*(k-1)+1,4*(k-1)+1,:))),'Color',color_wheel(1,:),'LineStyle','--')
    plot(input_tvec,-sigma*sqrt(squeeze(agents{j}.local_filter.cov_history(4*(k-1)+1,4*(k-1)+1,:))),'Color',color_wheel(1,:),'LineStyle','--','HandleVisibility','off')
    % fill([input_tvec flip(input_tvec)],[2*sqrt(squeeze(agents{1}.local_filter.cov_history(4*(N-1)+1,4*(N-1)+1,:))'), 2*sqrt(squeeze(agents{N}.local_filter.cov_history(1,1,:))')],'g','LineStyle','none')
    % alpha(0.25)

%     str1 = strcat(num2str(j),'estimating',num2str(k));
%     str2 = strcat(num2str(k),'estimating',num2str(j));
%     legend(str1,strcat(num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),str2,strcat(num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),strcat('cent. KF ',num2str(j)),strcat('cent. KF ',num2str(k)))

    plot(input_tvec,agents{k}.local_filter.state_history((j-1)*4+1,:)-agents{j}.true_state(1,:),'Color',[1 0 0])
    plot(input_tvec,sigma*sqrt(squeeze(agents{k}.local_filter.cov_history((j-1)*4+1,(j-1)*4+1,:))),'Color',[1 0 0],'LineStyle','--')
    plot(input_tvec,-sigma*sqrt(squeeze(agents{k}.local_filter.cov_history((j-1)*4+1,(j-1)*4+1,:))),'Color',[1 0 0],'LineStyle','--','HandleVisibility','off')
    % fill([input_tvec flip(input_tvec)],[2*sqrt(squeeze(agents{N}.local_filter.cov_history(1,1,:))'), -2*sqrt(squeeze(agents{N}.local_filter.cov_history(1,1,:))')],'r','LineStyle','none')
    % alpha(0.25)
    % plot(input_tvec,ci_time_vec,'x')

    plot(input_tvec,baseline_filter.state_history((j-1)*4+1,:)-agents{j}.true_state(1,:),'Color',color_wheel(5,:))
    plot(input_tvec,baseline_filter.state_history((k-1)*4+1,:)-agents{k}.true_state(1,:),'Color',color_wheel(3,:))
    % plot(input_tvec,2*sqrt(squeeze(baseline_filter.cov_history((j-1)*4+1,(j-1)*4+1,:))'),'k--')
    % plot(input_tvec,-2*sqrt(squeeze(baseline_filter.cov_history((j-1)*4+1,(j-1)*4+1,:))'),'k--')

    str1 = strcat(num2str(j),' estimating ',num2str(k));
    str2 = strcat(num2str(k),' estimating ',num2str(j));
%     legend(str1,strcat('\pm',num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),str2,strcat(num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),strcat('cent. KF  ',num2str(j)),strcat('cent. KF  ',num2str(k)))
    legend(str1,strcat('\pm',num2str(sigma),'\sigma'),str2,strcat('\pm',num2str(sigma),'\sigma'),strcat('cent. KF  ',num2str(j)),strcat('cent. KF  ',num2str(k)))
    xlabel('Time [s]')
    ylabel('Est Error [m]')
    title(['Estimation error in x pos, \delta=',num2str(delta),', ',num2str(j),'\leftrightarrow',num2str(k)])

else
    plot(input_tvec,agents{k}.local_filter.state_history((j-1)*4+1,:)-agents{j}.true_state(1,:),'Color',[1 0 0])
    plot(input_tvec,sigma*sqrt(squeeze(agents{k}.local_filter.cov_history((j-1)*4+1,(j-1)*4+1,:))),'Color',[1 0 0],'LineStyle','--')
    plot(input_tvec,-sigma*sqrt(squeeze(agents{k}.local_filter.cov_history((j-1)*4+1,(j-1)*4+1,:))),'Color',[1 0 0],'LineStyle','--','HandleVisibility','off')
    % fill([input_tvec flip(input_tvec)],[2*sqrt(squeeze(agents{N}.local_filter.cov_history(1,1,:))'), -2*sqrt(squeeze(agents{N}.local_filter.cov_history(1,1,:))')],'r','LineStyle','none')
    % alpha(0.25)
    % plot(input_tvec,ci_time_vec,'x')

    plot(input_tvec,baseline_filter.state_history((j-1)*4+1,:)-agents{j}.true_state(1,:),'Color',color_wheel(5,:))
    % plot(input_tvec,2*sqrt(squeeze(baseline_filter.cov_history((j-1)*4+1,(j-1)*4+1,:))'),'k--')
    % plot(input_tvec,-2*sqrt(squeeze(baseline_filter.cov_history((j-1)*4+1,(j-1)*4+1,:))'),'k--')

    str1 = strcat(num2str(j),' estimating ',num2str(k));
    str2 = strcat(num2str(k),' estimating ',num2str(j));
    legend(str2,strcat('\pm',num2str(sigma),'\sigma'),strcat('cent. KF  ',num2str(j)))
    xlabel('Time [s]')
    ylabel('Est Error [m]')
    title(['Estimation error in x pos, \delta=',num2str(delta),', ',num2str(j),'\leftrightarrow',num2str(k)])
end
    

% for i=1:size(ci_time_vec,2)
%     if ci_time_vec(1,i)
%         line([i*dt i*dt],get(gca,'ylim'));
%     end
% end

% subplot(2,1,2)
% hold on; grid on;
% 
% plot(input_tvec,agents{j}.local_filter.state_history(4*(k-1)+3,:) - agents{k}.true_state(3,:),'m')
% plot(input_tvec,sigma*sqrt(squeeze(agents{j}.local_filter.cov_history(4*(k-1)+3,4*(k-1)+3,:))),'g--')
% plot(input_tvec,-sigma*sqrt(squeeze(agents{j}.local_filter.cov_history(4*(k-1)+3,4*(k-1)+3,:))),'g--')
% 
% plot(input_tvec,agents{k}.local_filter.state_history(4*(j-1)+3,:)-agents{j}.true_state(3,:))
% plot(input_tvec,sigma*sqrt(squeeze(agents{k}.local_filter.cov_history(4*(j-1)+3,4*(j-1)+3,:))),'r--')
% plot(input_tvec,-sigma*sqrt(squeeze(agents{k}.local_filter.cov_history(4*(j-1)+3,4*(j-1)+3,:))),'r--')
% % plot(input_tvec,ci_time_vec,'x')
% 
% plot(input_tvec,baseline_filter.state_history(4*(j-1)+3,:)-agents{j}.true_state(3,:))
% plot(input_tvec,baseline_filter.state_history(4*(k-1)+3,:)-agents{k}.true_state(3,:))
% % plot(input_tvec,2*sqrt(squeeze(baseline_filter.cov_history(3,3,:))'),'k--')
% % plot(input_tvec,-2*sqrt(squeeze(baseline_filter.cov_history(3,3,:))'),'k--')
% 
% str1 = strcat('Agent ',num2str(j),' estimate of agent N=',num2str(k),' y pos');
% str2 = strcat('Agent N=',num2str(k),' estimate of agent ',num2str(j),' y pos');
% legend(str1,strcat(num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),str2,strcat(num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),strcat('cent. KF ',num2str(j)),strcat('cent. KF ',num2str(k)))
% ylabel('Est Error [m]')
% title(['Estimation error in y pos, \delta=',num2str(delta)])


% figure
% 
% subplot(2,1,1)
% hold on; grid on;
% 
% plot(input_tvec,agents{j}.local_filter.state_history(4*(k-1)+2,:) - agents{k}.true_state(2,:),'m')
% plot(input_tvec,2*sqrt(squeeze(agents{j}.local_filter.cov_history(4*(k-1)+2,4*(k-1)+2,:))),'g--')
% plot(input_tvec,-2*sqrt(squeeze(agents{j}.local_filter.cov_history(4*(k-1)+2,4*(k-1)+2,:))),'g--')
% % fill([input_tvec flip(input_tvec)],[2*sqrt(squeeze(agents{1}.local_filter.cov_history(4*(N-1)+1,4*(N-1)+1,:))'), 2*sqrt(squeeze(agents{N}.local_filter.cov_history(1,1,:))')],'g','LineStyle','none')
% % alpha(0.25)
% 
% plot(input_tvec,agents{k}.local_filter.state_history(4*(j-1)+2,:)-agents{j}.true_state(2,:))
% plot(input_tvec,2*sqrt(squeeze(agents{k}.local_filter.cov_history(4*(j-1)+2,4*(j-1)+2,:))),'r--')
% plot(input_tvec,-2*sqrt(squeeze(agents{k}.local_filter.cov_history(4*(j-1)+2,4*(j-1)+2,:))),'r--')
% % fill([input_tvec flip(input_tvec)],[2*sqrt(squeeze(agents{k}.local_filter.cov_history(1,1,:))'), -2*sqrt(squeeze(agents{N}.local_filter.cov_history(1,1,:))')],'r','LineStyle','none')
% % alpha(0.25)
% % plot(input_tvec,ci_time_vec,'x')
% 
% plot(input_tvec,baseline_filter.state_history(4*(j-1)+2,:)-agents{j}.true_state(2,:))
% plot(input_tvec,baseline_filter.state_history(4*(k-1)+2,:)-agents{k}.true_state(2,:))
% % plot(input_tvec,2*sqrt(squeeze(baseline_filter.cov_history(1,1,:))'),'k--')
% % plot(input_tvec,-2*sqrt(squeeze(baseline_filter.cov_history(1,1,:))'),'k--')
% 
% str1 = strcat('Agent ',num2str(j),' estimate of agent N=',num2str(k),' x vel');
% str2 = strcat('Agent N=',num2str(k),' estimate of agent ',num2str(j),' x vel');
% legend(str1,strcat(num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),str2,strcat(num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),strcat('cent. KF ',num2str(j)),strcat('cent. KF ',num2str(k)))
% xlabel('Time [s]')
% ylabel('Est Error [m/s]')
% title(['Estimation error in x vel, \delta=',num2str(delta)])
% 
% subplot(2,1,2)
% hold on; grid on;
% 
% plot(input_tvec,agents{j}.local_filter.state_history(4*(k-1)+4,:) - agents{k}.true_state(4,:),'m')
% plot(input_tvec,2*sqrt(squeeze(agents{j}.local_filter.cov_history(4*(k-1)+4,4*(k-1)+4,:))),'g--')
% plot(input_tvec,-2*sqrt(squeeze(agents{j}.local_filter.cov_history(4*(k-1)+4,4*(k-1)+4,:))),'g--')
% 
% plot(input_tvec,agents{k}.local_filter.state_history(4*(j-1)+4,:)-agents{j}.true_state(4,:))
% plot(input_tvec,2*sqrt(squeeze(agents{k}.local_filter.cov_history(4*(j-1)+4,4*(j-1)+4,:))),'r--')
% plot(input_tvec,-2*sqrt(squeeze(agents{k}.local_filter.cov_history(4*(j-1)+4,4*(j-1)+4,:))),'r--')
% % plot(input_tvec,ci_time_vec,'x')
% 
% plot(input_tvec,baseline_filter.state_history(4*(j-1)+4,:)-agents{j}.true_state(4,:))
% plot(input_tvec,baseline_filter.state_history(4*(k-1)+4,:)-agents{k}.true_state(4,:))
% % plot(input_tvec,2*sqrt(squeeze(baseline_filter.cov_history(3,3,:))'),'k--')
% % plot(input_tvec,-2*sqrt(squeeze(baseline_filter.cov_history(3,3,:))'),'k--')
% 
% str1 = strcat('Agent ',num2str(j),' estimate of agent N=',num2str(k),' y vel');
% str2 = strcat('Agent N=',num2str(k),' estimate of agent ',num2str(j),' y vel');
% legend(str1,strcat(num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),str2,strcat(num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),strcat('cent. KF ',num2str(j)),strcat('cent. KF ',num2str(k)))
% xlabel('Time [s]')
% ylabel('Est Error [m/s]')
% title(['Estimation error in y vel, \delta=',num2str(delta)])

end
