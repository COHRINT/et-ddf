% msg data post-processing
%
% Ian Loefgren
% 2.5.2019

abs_sent = zeros(2,N);
abs_total = zeros(2,N);
rel_sent = zeros(2,N);
rel_total = zeros(2,N);

% dim1=src agent, dim2=dest agent, dim3=meas type, dim4=element [x or y]
comms_mat_sent = zeros(N,N,2,2);
comms_mat_total = zeros(N,N,2,2);

for i=1:length(all_msgs)
    msg = all_msgs{i};
    
    type = msg.type=="rel";
    
    for j=1:length(msg.status)
        comms_mat_total(msg.src,msg.dest,type+1,j) = comms_mat_total(msg.src,msg.dest,type+1,j) + 1;
        if msg.status(j)
            comms_mat_sent(msg.src,msg.dest,type+1,j) = comms_mat_sent(msg.src,msg.dest,type+1,j) + 1;
        end
    end
    
end

msgs_sent_vec = zeros(1,N);
msgs_total_vec = zeros(1,N);
for i=1:length(agents)
    msgs_sent_vec(i) = agents{i}.msgs_sent;
    msgs_total_vec(i) = agents{i}.total_msgs;
end

msgs_sent_total = sum(msgs_sent_vec);
possible_msgs_total = sum(msgs_total_vec);
    
% comms heatmaps
figure
disp_mat = comms_mat_sent(:,:,1,1)./comms_mat_total(:,:,1,1);
disp_mat(isnan(disp_mat)) = 0;
heatmap(disp_mat)
title('Absolute msg x elements sent')

figure
disp_mat = comms_mat_sent(:,:,1,2)./comms_mat_total(:,:,1,2);
disp_mat(isnan(disp_mat)) = 0;
heatmap(disp_mat)
title('Absolute msg y elements sent')

figure
disp_mat = comms_mat_sent(:,:,2,1)./comms_mat_total(:,:,2,1);
disp_mat(isnan(disp_mat)) = 0;
heatmap(disp_mat)
title('Relative msg x elements sent')

figure
disp_mat = comms_mat_sent(:,:,2,2)./comms_mat_total(:,:,2,2);
disp_mat(isnan(disp_mat)) = 0;
heatmap(disp_mat)
title('Relative msg y elements sent')


%% CI trigger rates and total triggers

ci_trigger_vec = zeros(1,N);
usage_vec_ci = zeros(1,N);
usage_vec_ci_wmeans = zeros(1,N);
usage_vec_ci_fullstate = zeros(1,N);

for i=1:length(agents)
    ci_trigger_vec(1,i) = agents{i}.ci_trigger_cnt;
    usage_vec_ci_wmeans(1,i) = agents{i}.ci_trigger_cnt * (size(agents{i}.local_filter.x,1)^2 + size(agents{i}.local_filter.x,1)) * length(agents{i}.meas_connections);
    usage_vec_ci(1,i) = agents{i}.ci_trigger_cnt * (size(agents{i}.local_filter.x,1))^2 * length(agents{i}.meas_connections);
    usage_vec_ci_fullstate(1,i) = agents{i}.ci_trigger_cnt * ((4*N)^2 + (4*N)) * length(agents{i}.meas_connections);
end

figure
subplot(2,1,1)
heatmap(ci_trigger_vec)
title('Total times CI triggered by each agent')
xlabel('Agent id')
ylabel(' ')

subplot(2,1,2)
heatmap(ci_trigger_vec./((max_time/dt)))
title('CI trigger rate of each agent')
xlabel('Agent id')

ylabel(' ')
        

%% Total comms data transfer

% we quantify data transfer for each agent as the number of floats sent
% 1 measurement elemtent = 1 float = 1 unit
% 16x16 covar matrix = 16x16= 256 floats = 256 units

% each connection is treated seperately, meaning an agent with 3
% connections that triggers CI will share its stats 3 times

usage_vec_msg = sum(comms_mat_sent(:,:,1,1),2)' + sum(comms_mat_sent(:,:,1,2),2)' + sum(comms_mat_sent(:,:,2,1),2)' + sum(comms_mat_sent(:,:,1,2),2)';

usage_vec = usage_vec_ci_wmeans + usage_vec_msg;
usage_vec_fullstate = usage_vec_ci_fullstate + usage_vec_msg;

figure
% subplot(3,1,1)
heatmap(usage_vec_msg)
title('Total message data transfer by each agent')

figure
% subplot(3,1,2)
heatmap(usage_vec_ci/1000)
title('Total CI data transfer by each agent (x1000)')

figure
% subplot(3,1,3)
heatmap(usage_vec/1000)
title('Total data transfer by each agent (x1000)')

fprintf('Total usage: %i \n expected full state usage: %i\n',sum(usage_vec),sum(usage_vec_fullstate));
