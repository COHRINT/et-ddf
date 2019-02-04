% test for generating similarity transforms
%
% Ian Loefgren
% 1.29.2019

clc; close all; clear;

testA = randn(5,5);
testA = testA'*testA

Ta = gen_sim_transform(3,[1 2 5 8],8,[3 10 23],1);

testAT = inv(Ta)*testA*Ta

testAnew = Ta*testAT*inv(Ta);

testA == testAnew