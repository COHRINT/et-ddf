% Generate similarity transforms to collect common states at the beginning
% of state vectors
%
% Ian Loefgren
% 1.24.2019

function [T,il,inter] = gen_sim_transform(a_id,a_connections,b_id,b_connections,num_states)

    if nargin < 5
        num_states = 4;
    end

    a_conn = sort([a_connections,a_id]); % construct ordered list of all agent ids, including self and connections
    b_conn = sort([b_connections,b_id]);
    
    % compute intersection between states, as well as differences, and
    % indexes for both
    
    [inter,inter_idx] = intersect(a_conn,b_conn);
    [diff,diff_idx] = setdiff(a_conn,b_conn);
    
    idx = [inter_idx',diff_idx'];
    
    % compute number of intersecting platforms to be returned
    il = num_states*length(inter);
                
%     a_loc = find(a_conn == a_id); % find location in ordered list of self
%     b_loc = find(a_conn == b_id); % find location of target to fuse with
% 
%     a_conn_loc = 1:length(a_conn);
%     a_conn_loc(a_conn_loc == find(a_conn == a_id)) = []; % remove self agent id from list
%     a_conn_loc(a_conn_loc == find(a_conn == b_id)) = []; % remove fuse target id from list
% 
%     if a_loc < b_loc
%         a_conn_loc = [a_loc,b_loc,a_conn_loc]; % add removed ids back, at beginning of list
%     elseif b_loc < a_loc
%         a_conn_loc = [b_loc,a_loc,a_conn_loc];
%     end

%     idx = a_conn_loc;
    
    T = zeros(length(a_connections)+1);
    for ii=1:length(idx)
        T(num_states*(idx(ii)-1)+1:num_states*(idx(ii)-1)+num_states,...
            num_states*(ii-1)+1:num_states*(ii-1)+num_states) = eye(num_states);
    end
    
end