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
        method = 'opt';
    end
    
    % optimization-based scaling
    if strcmp(method,'opt')
        
        % vector of scale factors, c
        c = [];
        
        % D
        D = diag(diag(P));
        disp(D)
        
        % compute c* optimal scale factors
        fun = @(x) trace(diag(x)*D);
        
        c = @(x) -eig(diag(x)*D-P);
        ceq = @(x) [];
        constraint = @(x)deal(c(x),ceq(x));
        
        c_opt = fmincon(fun,ones(size(P,1),1),[],[],[],[],ones(size(P,1),1),10*ones(size(P,1),1),constraint,optimoptions('fmincon','Display','off'));
        
        % compute Dc
        Dc = diag(c_opt)*D;
    end


end

function [c,ceq] = PSDcon(X)

    [~,p] = chol(X);
    c = [];
    ceq = p;

end