%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Covariance matrix diagonalization
%
% Ian Loefgren
% Date: 7.26.2019
%
% Diagonalization of covariance matrices to reduces DDF data transfer.
% See [Forsling et. al., FUSION 2019] for methods implemented.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Dc] = covar_diagonalize(P,method)
    
    if nargin < 2
        method = 'eig';
    end
    
    % optimization-based scaling
    if strcmp(method,'opt')
        
        % D
        D = diag(diag(P));
        
        % compute c* optimal scale factors
        fun = @(x) trace(diag(x)*D);
        
        c = @(x) -eig(diag(x)*D-P);
        ceq = @(x) [];
        constraint = @(x)deal(c(x),ceq(x));
        
        c_opt = fmincon(fun,ones(size(P,1),1),[],[],[],[],ones(size(P,1),1),10*ones(size(P,1),1),constraint,optimoptions('fmincon','Display','off'));
        
        % compute Dc
        Dc = diag(c_opt)*D;
    elseif strcmp(method,'eig')
        
        % compute diagonal cov
        D = diag(diag(P));
        % compute correlation matrix Q
        Q = inv(sqrtm(D))*P*inv(sqrtm(D));
        % find largest eigenvalue of Q
        lambda = max(eig(Q));
        % inflate diagonal cov
        Dc = lambda*D;
        
    end


end

function [c,ceq] = PSDcon(X)

    [~,p] = chol(X);
    c = [];
    ceq = p;

end