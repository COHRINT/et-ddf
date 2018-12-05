% CT to DT conversion
%
% Ian Loefgren
% Last modified: 11.11.2018
%
% Convert CT LTI systems to DT LTI systems

function [F,G,H,M,Q,R] = ct2dt(A,B,C,D,Gamma,Qct,Rct,dt)

% compute block matrix exponential
Ahat = [A B;
        zeros(size(B,2),size(A,1)) zeros(size(B,2))];
soln = expm(Ahat*dt);
F = soln(1:size(A,1),1:size(A,2));
G = soln(1:size(B,1),1+size(A,2):end);

% output equations are algebraic
H = C;
M = D;

% using Van Loan's method to find process noise covariance
Z = dt*[-A Gamma*Qct*Gamma';
        zeros(size(A)) A'];
soln = expm(Z);
invFQ = soln(1:size(A,1), end-size(A,2)+1:end); 
Q = F*invFQ;

% dt measurement noise computation
R = (Rct/dt)*eye(size(C,1));

end