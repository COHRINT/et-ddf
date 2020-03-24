#!/usr/bin/env python
# -*- encoding: utf-8 -*-
from __future__ import division
import numpy as np
from scipy.stats import norm as normal
from asset import Asset
from measurements import *

# Simple simulation
K = 1000
dim = 1
A = np.eye(dim)
B = np.eye(dim)
q = 2
Q = np.eye(dim) * (q ** 2)
r = 1
R = np.eye(dim) * (r ** 2)

# Estimates
initial_x = np.array([[0]]).T
initial_P = np.zeros((1,1)) 
# robotA = Robot( initial_x, initial_P )
# robotB = Robot( initial_x, initial_P )
# robots = [robotA] # robotB

delta = 5
x_truth = np.array([[0]]).T

implicit_update_cnt = 0

asset = Asset(0, initial_x, initial_P, A, B, delta)

for k in range(K):
    u = np.random.randint(-10, 10, size=(dim,1))
    w = np.random.normal(0, q, size=(dim,1))

    # Truth update
    x_truth = A.dot(x_truth) + B.dot(u) + w
    meas = x_truth + np.random.multivariate_normal( np.zeros((dim,)), R).reshape(-1,1)

    # TODO see if we get increased accuracy with range measurements
    # TODO 3 robots with relative range measurements
    # TODO 3 robots, 2 with only relative range, 1 with GPS (that'll be really interesting)
    # TODO implement covariance intersection with above when uncertainty gets above some bound!

    asset.predict(u, Q)
    meas = GPS_Explicit(meas, R)
    asset.receive_meas(meas)
    asset.correct()
    
print("Truth: \n" + str(x_truth))
print("x_hat: \n" + str(asset.main_filter.x_hat))
print("P: \n" + str(asset.main_filter.P))
print("Uncertainty sigma: \n" + str(2*np.sqrt(asset.main_filter.P)))
# uncertainty_range = [x_hat - 2*np.sqrt(P), x_hat + 2*np.sqrt(P)]
# print("Uncertainty range: " + str(uncertainty_range))
# print("Inside range? " + str(x > uncertainty_range[0] and x < uncertainty_range[1]))
# print("Num implicit percentage: " + str(implicit_update_cnt / K ))