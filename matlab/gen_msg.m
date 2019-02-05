% Function to generate messages for ET-DDF matlab framework
%
% Ian Loefgren
% 2.4.2019

function [msg] = gen_msg(src_id,dest_id,status,type,data)

% type checking for each msg component
% assert(isinteger(src_id) == true);
% assert(isinteger(dest_id) == true);
% assert(islogical(status(1)) == true);
% assert(isstring(type) == true);
% assert(isfloat(data(1)) == true);


% make sure source id is not the same as destination id
assert(src_id ~= dest_id);

% check that number of transmited elements is equal to data length
if ~isempty(status)
    assert(sum(status) == length(data));
end

msg = struct('src',src_id,'dest',dest_id,'status',status,'type',type,'data',data);

end