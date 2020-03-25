#!/usr/bin/env python
# -*- encoding: utf-8 -*-
from __future__ import division
import numpy as np
from scipy.stats import norm as normal
from asset import Asset
from measurements import *

# Simple simulation
K = 1000
dim = 2
A = np.eye(dim)
B = np.eye(dim)
q = 2
Q = np.eye(dim) * (q ** 2)
r = 1
R = np.eye(dim) * (r ** 2)

# Estimates
x_truth = np.array([[0, 5]]).T
initial_P = np.zeros((dim,dim)) 
# robotA = Robot( initial_x, initial_P )
# robotB = Robot( initial_x, initial_P )
# robots = [robotA] # robotB

delta = 5


implicit_update_cnt = 0

asset0 = Asset(0, x_truth, initial_P, A, B, delta)
asset1 = Asset(1, x_truth, initial_P, A, B, delta)

for k in range(K):
    u = np.random.randint(-1, 1, size=(dim,1))
    # u = np.zeros((dim,1))
    w = np.random.normal(0, q, size=(dim,1))

    # Truth update
    x_truth = A.dot(x_truth) + B.dot(u) + w
    meas = x_truth + np.random.multivariate_normal( np.zeros((dim,)), R).reshape(-1,1)
    relative_dist_01 = x_truth[1,0] - x_truth[0,0] + np.random.normal(0, r)
    diff_meas = LinRelx_Explicit(relative_dist_01, 1, r)

    asset0.predict(u, Q)
    asset1.predict(u, Q)

    # Asset 0 can get distance to other asset
    shared_meas = asset0.receive_meas(diff_meas, shareable=True)
    asset1.receive_shared_meas(0, shared_meas[1])
    
    # Asset 1 has GPS
    meas_gps2 = GPSx_Explicit(meas[1,0], r)
    shared_meas2 = asset1.receive_meas(meas_gps2, shareable=True)
    if isinstance(shared_meas2[0], Implicit):
        implicit_update_cnt += 1
    asset0.receive_shared_meas(1, shared_meas2[0])

    # By sharing measurements, asset0's position becomes fully observable
    asset0.correct()
    asset1.correct()

print("Truth: \n" + str(x_truth))

asset0.print_filters(main_only=True)
asset1.print_filters(main_only=True)
# asset0.print_filters()
# asset1.print_filters()
print("---------------------------------------------------------------")
# print("x_hat: \n" + str(asset0.main_filter.x_hat))
# print("P: \n" + str(asset0.main_filter.P))
# print("Uncertainty sigma: \n" + str(2*np.sqrt(asset0.main_filter.P)))

# uncertainty_range = [x_hat - 2*np.sqrt(P), x_hat + 2*np.sqrt(P)]
# print("Uncertainty range: " + str(uncertainty_range))
# print("Inside range? " + str(x > uncertainty_range[0] and x < uncertainty_range[1]))
print("Num implicit percentage: " + str(implicit_update_cnt / (2*K) ))