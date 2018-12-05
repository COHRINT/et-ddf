% NEES function test script
% 11.20.2018

clear; clc; close all;

% define system

% CT model
A = [0 1 0 0;
    0 0 0 0;
    0 0 0 1;
    0 0 0 0];
B = [0 0;
    1 0;
    0 0;
    0 1];
Gamma = [0 0;
         1 0;
         0 0;
         0 1];

C = [1 0 0 0;
    0 0 1 0];

D = 0;

Qct = 3.5; % additive white gaussian process noise intensity [m/s/s]^2
Rct = 1; % additive white gaussian measurement noise intensity [m^2]

% convert to DT model
dt = 0.1;

[F,G,H,M,Q,R] = ct2dt(A,B,C,D,Gamma,Qct,Rct,dt);

% create filter object
x0 = zeros(size(F,1),1);
P0 = eye(size(F));
kf = KF(F,G,H,M,Q,R,x0,P0);

% run nees
consistent = nees(kf,0.1);