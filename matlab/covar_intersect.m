% Covariance Intersection
%
% Ian Loefgren
% 11.16.2018
%
% Computes a conservative, consistent fusion of two covariance matrices and
% associated means. See Julier, Uhlmann. "A Non-divergent Estimation
% Algorithm in the Presence of Unknown Correlations" 1997.
%
% Usage:
%   xa - mean A, must be column vector
%   xb - mean B, must be column vector
%   Pa - covariance of A
%   Pb - covariance of B
%
% Returns:
%   xc - fused mean
%   Pc - fused covariance
%

function [xc,Pc] = covar_intersect(xa,xb,Pa,Pb,alpha)

if nargin < 5
    alpha = ones(size(xa,1),1);
end

% create fxn handle to find omega that minimizes tr(P)
f = @(omega) trace(inv(omega*inv(Pa) + (1-omega)*inv(Pb))*diag(alpha));
omega = gss(f,0,1);

Pc = inv(omega*inv(Pa) + (1-omega)*inv(Pb));
xc = Pc*(omega*inv(Pa)*xa + (1-omega)*inv(Pb)*xb);

end