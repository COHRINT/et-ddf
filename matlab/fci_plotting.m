%% estimation error plots
num_agents = N;
sigma = 2;
num_states = 4;
statenum2label = {'x','xdot','y','ydot'};

% agent_plots_1 = [18 16 3 13 13 22];
% agent_plots_2 = [18 18 10 13 15 30];
% agent_plots_1 = [1 2 6 3 4 4];
% agent_plots_2 = [3 3 6 4 5 6];
% agent_plots_1 = [1 2 3 4 4];
% agent_plots_2 = [3 3 4 5 6];
% agent_plots_1 = [1 1 2 3 2];
% agent_plots_2 = [2 4 6 8 3];
% agent_plots{1} = [1 2 2 3 3 4;
%                     2 2 3 3 4 4];
% agent_plots{2} = [4 5 5 6;5 5 6 6];
agent_plots{1} = [1 2 3 4 5 6;
                    1 2 3 4 5 6];
agent_plots{2} = [1 2 3 4 4;
                    3 3 4 5 6];
% agent_plots{1} = [1 2; 1 2];
% agent_plots{2} = [1 2;2 1];
% agent_plots{1} = [18 16 4 13 13 22;
%                     18 18 10 13 15 30];
% agent_plots{2} = [4 10 15 16 22 30;
%                     4 10 15 16 22 30];

color_wheel = [0    0.4470    0.7410;
    0.8500    0.3250    0.0980;
    0.9290    0.6940    0.1250;
    0.4940    0.1840    0.5560;
    0.4660    0.6740    0.1880;
    0.3010    0.7450    0.9330;
    0.6350    0.0780    0.1840];

if N==30

% agent connectivity groups
conngroup1 = 1:8;
conngroup2 = 9:12;
conngroup3 = 13:15;
conngroup4 = 16:18;
conngroup5 = 19:22;
conngroup6 = 23:30;

figure
subplot(3,2,1)
hold on; grid on;
for i=conngroup1
    plot(input_tvec,agents{i}.tau_history(:));
end
legend('1','2','3','4','5','6','7','8')
xlabel('Time [s]')
ylabel('\tau')
title('CI threshold for connectivity group 1: 1-8')

subplot(3,2,2)
hold on; grid on;
for i=conngroup2
    plot(input_tvec,agents{i}.tau_history(:));
end
legend('9','10','11','12')
xlabel('Time [s]')
ylabel('\tau')
title('CI threshold for connectivity group 2: 9-12')

subplot(3,2,3)
hold on; grid on;
for i=conngroup3
    plot(input_tvec,agents{i}.tau_history(:));
end
legend('13','14','15')
xlabel('Time [s]')
ylabel('\tau')
title('CI threshold for connectivity group 3: 13-15')

subplot(3,2,4)
hold on; grid on;
for i=conngroup4
    plot(input_tvec,agents{i}.tau_history(:));
end
legend('16','17','18')
xlabel('Time [s]')
ylabel('\tau')
title('CI threshold for connectivity group 4: 16-18')

subplot(3,2,5)
hold on; grid on;
for i=conngroup5
    plot(input_tvec,agents{i}.tau_history(:));
end
legend('19','20','21','22')
xlabel('Time [s]')
ylabel('\tau')
title('CI threshold for connectivity group 5: 19-22')

subplot(3,2,6)
hold on; grid on;
for i=conngroup6
    plot(input_tvec,agents{i}.tau_history(:));
end
legend('23','24','25','26','27','28','29','30')
xlabel('Time [s]')
ylabel('\tau')
title('CI threshold for connectivity group 6: 23-30')

end

% figure
% subplot(2,2,1)
% hold on; grid on;
% plot(agents{18}.local_filter.innovation_history(1,:))
% plot(agents{18}.local_filter.innovation_history(2,:))
% legend('x component','y component')
% title('Robot 18 local estimate innov')
% 
% subplot(2,2,2)
% hold on; grid on;
% plot(agents{22}.local_filter.innovation_history(1,:))
% plot(agents{22}.local_filter.innovation_history(2,:))
% legend('x component','y component')
% title('Robot 22 local estimate innov')
% 
% subplot(2,2,3)
% hold on; grid on;
% plot(agents{18}.common_estimates{1}.innovation_history(1,:))
% plot(agents{18}.common_estimates{1}.innovation_history(2,:))
% legend('x component','y component')
% title('Robot 18 common estimate innov')
% 
% subplot(2,2,4)
% hold on; grid on;
% plot(agents{22}.common_estimates{1}.innovation_history(1,:))
% plot(agents{22}.common_estimates{1}.innovation_history(2,:))
% legend('x component','y component')
% title('Robot 22 common estimate innov')

ci_trigger_mat_plotting = ci_trigger_mat;
ci_trigger_mat_plotting(ci_trigger_mat_plotting == 0) = NaN;

abs_meas_mat(abs_meas_mat == 0) = NaN;

figure
heatmap(ci_trigger_mat);

for state_num=1

for ii=1:length(agent_plots)

figure
for i=1:length(agent_plots{ii})
    
j = agent_plots{ii}(1,i);
k = agent_plots{ii}(2,i);

j_loc = find(sort([agents{k}.connections,agents{k}.agent_id]) == agent_plots{ii}(1,i));
k_loc = find(sort([agents{j}.connections,agents{j}.agent_id]) == agent_plots{ii}(2,i));



% if i>3
figure
% end

% subplot(2,3,i)
hold on; grid on;

set(gca,'FontSize',13)

if j~=k
    plot(input_tvec,agents{j}.local_filter.state_history(4*(k_loc-1)+state_num,:) - agents{k}.true_state(state_num,:),'Color',color_wheel(1,:))
    plot(input_tvec,sigma*sqrt(squeeze(agents{j}.local_filter.cov_history(4*(k_loc-1)+state_num,4*(k_loc-1)+state_num,:))),'Color',color_wheel(1,:),'LineStyle','--')
    plot(input_tvec,-sigma*sqrt(squeeze(agents{j}.local_filter.cov_history(4*(k_loc-1)+state_num,4*(k_loc-1)+state_num,:))),'Color',color_wheel(1,:),'LineStyle','--','HandleVisibility','off')
    % fill([input_tvec flip(input_tvec)],[2*sqrt(squeeze(agents{1}.local_filter.cov_history(4*(N-1)+1,4*(N-1)+1,:))'), 2*sqrt(squeeze(agents{N}.local_filter.cov_history(1,1,:))')],'g','LineStyle','none')
    % alpha(0.25)

%     str1 = strcat(num2str(j),'estimating',num2str(k));
%     str2 = strcat(num2str(k),'estimating',num2str(j));
%     legend(str1,strcat(num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),str2,strcat(num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),strcat('cent. KF ',num2str(j)),strcat('cent. KF ',num2str(k)))

    plot(input_tvec,agents{k}.local_filter.state_history((j_loc-1)*4+state_num,:)-agents{j}.true_state(state_num,:),'Color',[1 0 0])
    plot(input_tvec,sigma*sqrt(squeeze(agents{k}.local_filter.cov_history((j_loc-1)*4+state_num,(j_loc-1)*4+state_num,:))),'Color',[1 0 0],'LineStyle','--')
    plot(input_tvec,-sigma*sqrt(squeeze(agents{k}.local_filter.cov_history((j_loc-1)*4+state_num,(j_loc-1)*4+state_num,:))),'Color',[1 0 0],'LineStyle','--','HandleVisibility','off')
    % fill([input_tvec flip(input_tvec)],[2*sqrt(squeeze(agents{N}.local_filter.cov_history(1,1,:))'), -2*sqrt(squeeze(agents{N}.local_filter.cov_history(1,1,:))')],'r','LineStyle','none')
    % alpha(0.25)
    % plot(input_tvec,ci_time_vec,'x')

    plot(input_tvec,baseline_filter.state_history((j-1)*4+state_num,:)-agents{j}.true_state(state_num,:),'Color',color_wheel(5,:))
    plot(input_tvec,baseline_filter.state_history((k-1)*4+state_num,:)-agents{k}.true_state(state_num,:),'Color',color_wheel(3,:))
%     plot(input_tvec,2*sqrt(squeeze(baseline_filter.cov_history((j-1)*4+state_num,(j-1)*4+state_num,:))'),'--','Color',color_wheel(5,:))
%     plot(input_tvec,-2*sqrt(squeeze(baseline_filter.cov_history((j-1)*4+state_num,(j-1)*4+state_num,:))'),'--','Color',color_wheel(5,:))
%     plot(input_tvec,2*sqrt(squeeze(baseline_filter.cov_history((k-1)*4+state_num,(k-1)*4+state_num,:))'),'--','Color',color_wheel(3,:))
%     plot(input_tvec,-2*sqrt(squeeze(baseline_filter.cov_history((k-1)*4+state_num,(k-1)*4+state_num,:))'),'--','Color',color_wheel(3,:))

    str1 = strcat(num2str(j),' estimating ',num2str(k));
    str2 = strcat(num2str(k),' estimating ',num2str(j));
%     legend(str1,strcat('\pm',num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),str2,strcat(num2str(sigma),'\sigma'),strcat('-',num2str(sigma),'\sigma'),strcat('cent. KF  ',num2str(j)),strcat('cent. KF  ',num2str(k)))
    l = legend(str1,strcat('\pm',num2str(sigma),'\sigma'),str2,strcat('\pm',num2str(sigma),'\sigma'),strcat('cent. KF  ',num2str(j)),strcat('cent. KF  ',num2str(k)));
    l.FontSize = 10;
    xlabel('Time [s]')
    ylabel('Est Error [m]')
    title(['Estimation error in ',num2str(state_num),', \delta=',num2str(delta),', ',num2str(j),'\leftrightarrow',num2str(k)])
    
%     plot(input_tvec,ci_trigger_mat_plotting(j,:),'kx')
%     plot(input_tvec,ci_trigger_mat_plotting(k,:),'ko')
    
%     if state_num==1
%         plot(input_tvec,abs_meas_mat(j,:,1)-agents{j}.true_state(1,:),'bx')
%         plot(input_tvec,abs_meas_mat(k,:,1)-agents{k}.true_state(1,:),'rx')
%     elseif state_num==3
%         plot(input_tvec,abs_meas_mat(j,:,2)-agents{j}.true_state(3,:),'bx')
%         plot(input_tvec,abs_meas_mat(k,:,2)-agents{k}.true_state(3,:),'rx')
%     end

else
    plot(input_tvec,agents{k}.local_filter.state_history((j_loc-1)*4+state_num,:)-agents{j}.true_state(state_num,:),'Color',[1 0 0])
    plot(input_tvec,sigma*sqrt(squeeze(agents{k}.local_filter.cov_history((j_loc-1)*4+state_num,(j_loc-1)*4+state_num,:))),'Color',[1 0 0],'LineStyle','--')
    plot(input_tvec,-sigma*sqrt(squeeze(agents{k}.local_filter.cov_history((j_loc-1)*4+state_num,(j_loc-1)*4+state_num,:))),'Color',[1 0 0],'LineStyle','--','HandleVisibility','off')
    % fill([input_tvec flip(input_tvec)],[2*sqrt(squeeze(agents{N}.local_filter.cov_history(1,1,:))'), -2*sqrt(squeeze(agents{N}.local_filter.cov_history(1,1,:))')],'r','LineStyle','none')
    % alpha(0.25)
    % plot(input_tvec,ci_time_vec,'x')

    plot(input_tvec,baseline_filter.state_history((j-1)*4+state_num,:)-agents{j}.true_state(state_num,:),'Color',color_wheel(5,:))
%     plot(input_tvec,2*sqrt(squeeze(baseline_filter.cov_history((j-1)*4+state_num,(j-1)*4+state_num,:))'),'--','Color',color_wheel(5,:))
%     plot(input_tvec,-2*sqrt(squeeze(baseline_filter.cov_history((j-1)*4+state_num,(j-1)*4+state_num,:))'),'--','Color',color_wheel(5,:))

    str1 = strcat(num2str(j),' estimating ',num2str(k));
    str2 = strcat(num2str(k),' estimating ',num2str(j));
    l = legend(str2,strcat('\pm',num2str(sigma),'\sigma'),strcat('cent. KF  ',num2str(j)));
    l.FontSize = 10;
    xlabel('Time [s]')
    ylabel('Est Error [m]')
    title(['Estimation error in ',num2str(state_num),', \delta=',num2str(delta),', ',num2str(j),'\leftrightarrow',num2str(k)])
    
%     plot(input_tvec,ci_trigger_mat_plotting(j,:),'kx')
    
%     if state_num==1
%         plot(input_tvec,abs_meas_mat(j,:,1)-agents{j}.true_state(1,:),'rx')
%     elseif state_num==3
%         plot(input_tvec,abs_meas_mat(j,:,2)-agents{j}.true_state(3,:),'rx')
%     end
end

end

end

end