%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Golden Section Search
%
% Ian Loefgren
% 11.25.2018
%
% Golden section search to minimize a 1-D function. 
% See https://en.wikipedia.org/wiki/Golden-section_search
%
% Usage:
%   f - function handle for function to minimize
%   a - left bound for search
%   b - right bound for search
%   tol_ (optional) - tolerance for search, default 1E-5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function min = gss(f,a,b,tol_)

if nargin > 3
    tol = tol_;
else
    tol = 1E-5;
end

% golden ratio
gr = (1+sqrt(5))/2;

c = b-(b-a)/gr;
d = a+(b-a)/gr;

while abs(c-d) > tol
    if f(c) < f(d)
        b = d;
    else
        a = c;
    end
    
    c = b-(b-a)/gr;
    d = a+(b-a)/gr;
    
min = (a+b)/2;


end