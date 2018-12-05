% covariance intersection testing
%
% 11.16.2018
%
%

clc; clear; close all;

% Pa = [100 40;
%     40 50];
% 
% Pb = [400 100;
%     100 100];

Pa = [100 0; 0 5];
Pb = [5 0; 0 100];

% create fxn handle to find omega that minimizes tr(P)
f = @(omega) trace(inv(omega*inv(Pa) + (1-omega)*inv(Pb)));
omega = gss(f,0,1);

Pc = inv(omega*inv(Pa) + (1-omega)*inv(Pb));

figure
hold on; grid on;
p1 = error_ellipse(Pc);
p2 = error_ellipse(Pa);
p3 = error_ellipse(Pb);