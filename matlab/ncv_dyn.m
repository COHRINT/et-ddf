% NCV Dyanmics
%
% Ian Loefgren
% 1.11.2019
%
% Generates nearly constant velocity dynamics matrices for a platform with the 
% state vector [xpos,xvel,ypos,yvel]^T
%
% Usage:
%   dt = timestep
%   (optional) n = number of platforms, default = 1

function [F,G] = ncv_dyn(dt,n)

    if nargin < 2
        n = 1;
    end

    f = [1 dt 0 0;
        0 1 0 0;
        0 0 1 dt;
        0 0 0 1];
    
    g = [0 0;
        1 0;
        0 0;
        0 1];

    F = [];
    G = [];
    for i=1:n
        F = blkdiag(F,f);
        G = blkdiag(G,g);
    end
    

end