#!/usr/bin/env python
# -*- encoding: utf-8 -*-
from __future__ import division
import numpy as np
from scipy.stats import norm as normal
from asset import Asset
from measurements import *

# Simple simulation
K = 10
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

delta = 3


implicit_update_cnt = 0

asset0 = Asset(0, x_truth, initial_P, A, B, delta)
asset1 = Asset(1, x_truth, initial_P, A, B, delta)

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

    asset0.predict(u, Q)
    asset1.predict(u, Q)

    meas_gps = GPSx_Explicit(meas[0,0], r)
    shared_meas = asset0.receive_meas(meas_gps, shareable=True)
    print(shared_meas)
    if isinstance(shared_meas[1], Implicit):
        implicit_update_cnt += 1
    asset1.receive_shared_meas(0, shared_meas[1])

    meas_gps = GPSx_Explicit(meas[1,0], r)
    asset1.receive_meas(meas_gps, shareable=False)
    # asset0.receive_shared_meas(1, shared_meas)


    asset0.correct()
    asset1.correct()

    print("Truth: \n" + str(x_truth))
    # asset0.print_filters()
    asset1.print_filters()
# print("x_hat: \n" + str(asset0.main_filter.x_hat))
# print("P: \n" + str(asset0.main_filter.P))
# print("Uncertainty sigma: \n" + str(2*np.sqrt(asset0.main_filter.P)))

# uncertainty_range = [x_hat - 2*np.sqrt(P), x_hat + 2*np.sqrt(P)]
# print("Uncertainty range: " + str(uncertainty_range))
# print("Inside range? " + str(x > uncertainty_range[0] and x < uncertainty_range[1]))
# print("Num implicit percentage: " + str(implicit_update_cnt / K ))