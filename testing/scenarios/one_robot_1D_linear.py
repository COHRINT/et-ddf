#!/usr/bin/env python
# -*- encoding: utf-8 -*-
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from scipy.stats import norm as normal
from asset import Asset
from measurements import *
from copy import deepcopy
from error_plotting import *
from etdynamics import *

from pdb import set_trace

# np.random.seed(1)

DEBUG = False

# Simple simulation
K = 1000
world_dim = 1
num_assets = 1
num_ownship_states = 1
num_states = num_ownship_states * num_assets

def main():

    ######## DEFINE STATE && UNCERTAINTY #############
    x_truth = np.array([[0]])
    P_initial = np.array([2])

    ####### DEFINE PROCESS NOISE #######
    Q = np.array([0.1])

    ####### DEFINE PERCEIVED PROCESS NOISE #######
    Q_perceived = np.array([0.3])

    ###### DEFINE DYNAMICS #########
    linear_dynamics_status = True

    ########## DEFINE MEAS NOISE ##########
    R = np.array([0.1])

    ########## INITIALIZE ASSETS ##########
    asset = Asset(0, num_ownship_states, world_dim, x_truth, P_initial, linear_dynamics_status)

    ########## DEFINE ET DELTAS ##########
    gps_xy_delta = 3

    ########## DATA RECORDING ##########
    x_truth_bag = np.array([])
    x_hat_bag0 = np.array([])
    p_bag0 = np.array([])
    
    seq = 0

    for k in range(K):
        ########## DEFINE CONTROL INPUT ##########
        u = np.array([0.1])

        ########## SIMULATE TRUTH MOTION ##########
        x_truth_prev = deepcopy(x_truth)
        x_truth = linear_propagation(x_truth, u, world_dim, num_ownship_states)

        ########## ADD NOISE TO TRUTH MOTION ##########
        x_truth_no_noise = deepcopy(x_truth)
        x_truth += np.random.normal(0, np.sqrt(Q))

        ########## PREDICTION STEP  ##########
        asset.predict(u, Q_perceived)

        ########## GENERATE MEASUREMENTS VALUES ##########
        meas_vector = x_truth

        ########## ADD NOIES TO MEASUREMENTS ##########
        meas_vector += np.random.normal(0, np.sqrt(R))
    
        ########## INITIALIZE MEASUREMENT TYPES ##########
        gpsx0 = GPSx_Explicit(0, meas_vector, R, gps_xy_delta)

        ########## ASSETS RECEIVE MEASUREMNTS  ##########
        asset.receive_meas(gpsx0, shareable=False)

        ########## ASSETS SHARE MEASUREMENTS  ##########

        
        # asset.receive_meas(gpsy0, shareable=False)
        # asset.receive_meas(gpsyaw0, shareable=False)
        # asset1.receive_meas(gpsx1, shareable=False)
        # asset1.receive_meas(gpsy1, shareable=False)
        # asset1.receive_meas(gpsyaw1, shareable=False)

        # sharing = []
        # sharing.append(asset.receive_meas(gpsx0, shareable=True))
        # sharing.append(asset.receive_meas(gpsy0, shareable=True))
        # sharing.append(asset.receive_meas(gpsyaw0, shareable=True))
        # sharing.append(asset1.receive_meas(gpsx1, shareable=True))
        # sharing.append(asset1.receive_meas(gpsy1, shareable=True))
        # sharing.append(asset1.receive_meas(gpsyaw1, shareable=True))

        # sharing.append(asset.receive_meas(diff_measx, shareable=True))
        # sharing.append(asset.receive_meas(diff_measy, shareable=True))

        # sharing.append(asset1.receive_meas(diff_measx_red, shareable=True))
        # sharing.append(asset1.receive_meas(diff_measy_red, shareable=True))
        

        # for s in sharing:
        #     i = s.keys()[0]
        #     if isinstance(s[i], Implicit):
        #         implicit_update_cnt += 1
        #     total_num_meas_cnt += 2
        #     a = asset_list[i]
        #     meas = s[i]
        #     # print("Asset "+ str(meas.src_id) + " sharing with " + str(i) + " " + s[i].__class__.__name__ + "| data: " + str(meas.data))
        #     a.receive_shared_meas(meas)

        ########## CORRECTION STEP ##########
        asset.correct()

        ########## RECORD DATA ##########
        if x_truth_bag.size == 0:
            x_truth_bag = deepcopy(x_truth).reshape(-1,1)
        else:
            x_truth_bag = np.concatenate((x_truth_bag, x_truth.reshape(-1,1)), axis=1)

        # Estimate 0
        if x_hat_bag0.size == 0:
            x_hat_bag0 = deepcopy(asset.main_filter.x_hat).reshape(-1,1)
        else:
            x_hat_bag0 = np.concatenate((x_hat_bag0, asset.main_filter.x_hat), axis=1)

        # Uncertainty 0
        if p_bag0.size == 0:
            p_bag0 = deepcopy(asset.main_filter.P).reshape(num_states, num_states)
        else:
            p_bag0 = np.concatenate((p_bag0, asset.main_filter.P.reshape(num_states,num_states)), axis=1)

        ########## DEBUG FILTER INPUTS ##########
        if DEBUG:
            print("Filter Debug Step: " + str(seq))
            # Check what our error is at this step
            set_trace()

        seq += 1
        print(str(seq) + " out of " + str(K))
    
    plot_error(x_truth_bag, x_hat_bag0, p_bag0, num_ownship_states, 0)            

def normalize_angle(angle):
    return np.mod( angle + np.pi, 2*np.pi) - np.pi

if __name__ == "__main__":
    main()