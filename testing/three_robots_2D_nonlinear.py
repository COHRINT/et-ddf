#!/usr/bin/env python
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
from copy import deepcopy

from pdb import set_trace

np.random.seed(1)

DEBUG = False

# Simple simulation
K = 500
world_dim = 2
num_assets = 3
num_ownship_states = 6
num_states = num_ownship_states * num_assets

def main():

    ######## DEFINE STATE && UNCERTAINTY #############
    x_truth = np.array([[0,0,0,0,0,0,
                        0,7.5,np.pi/4,0,0,0,
                        5,-2,np.pi/2,0,0,0]], dtype=np.float64).T
    P_initial = np.array([[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,100,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,100,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],], dtype=np.float64)

    ####### DEFINE PROCESS NOISE #######
    q = 0.01
    q_yaw = 0.01

    ####### DEFINE PERCEIVED PROCESS NOISE #######
    Q_perceived = np.array([[2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],], dtype=np.float64)
    ###### DEFINE DYNAMICS #########
    linear_dynamics_status = False

    ########## DEFINE MEAS NOISE ##########
    r_gps = 0.3
    r_gps_yaw = 0.1
    r_gps_perceived = 0.5
    r_gps_yaw_perceived = 0.1

    r_range = 0.1
    r_bearing = 0.1
    r_range_perceived = 0.3
    r_bearing_perceived = 0.1
    

    ########## DEFINE ET DELTAS ##########
    gps_yaw_delta = 0.1
    gps_xy_delta = 0.2

    ########## INITIALIZE ASSETS ##########
    asset_list = []
    asset = Asset(0, num_ownship_states, world_dim, x_truth, P_initial, linear_dynamics_status, red_team=[2])
    asset1 = Asset(1, num_ownship_states, world_dim, x_truth, P_initial, linear_dynamics_status, red_team=[2])
    asset_list.append(asset); asset_list.append(asset1)

    ########## DATA RECORDING ##########
    x_truth_bag = x_truth_bag = deepcopy(x_truth).reshape(-1,1)
    x_hat_bag0 = deepcopy(asset.main_filter.x_hat).reshape(-1,1)
    p_bag0 = deepcopy(asset.main_filter.P)
    x_hat_bag1 = deepcopy(asset1.main_filter.x_hat).reshape(-1,1)
    p_bag1 = deepcopy(asset1.main_filter.P)
    
    seq = 0
    implicit_update_cnt = 0
    total_num_meas_cnt = 0
    mf0_xhat = deepcopy(asset.main_filter.x_hat)
    mf0_P = deepcopy(asset.main_filter.P)
    mf1_xhat = deepcopy(asset1.main_filter.x_hat)
    mf1_P = deepcopy(asset1.main_filter.P)
    plotting_bag = [[deepcopy(x_truth), mf0_xhat, mf0_P, mf1_xhat, mf1_P]]

    for k in range(K):
        ########## DEFINE CONTROL INPUT ##########
        u0 = np.array([[0.25, np.pi/50]], dtype=np.float64).T
        u1 = np.array([[0.25, np.pi/50]], dtype=np.float64).T
        # u0 = np.array([[0.25, 0]], dtype=np.float64).T
        # u1 = np.array([[0.25, 0]], dtype=np.float64).T
        u2 = np.array([[0.1, 0]], dtype=np.float64).T

        ########## SIMULATE TRUTH MOTION ##########
        x_truth0 = nonlinear_propagation(x_truth, u0, world_dim, num_ownship_states, 0)
        x_truth0[2,0] = normalize_angle(x_truth0[2,0])
        x_truth1 = nonlinear_propagation(x_truth, u1, world_dim, num_ownship_states, 1)
        x_truth1[2,0] = normalize_angle(x_truth1[2,0])
        x_truth2 = nonlinear_propagation(x_truth, u2, world_dim, num_ownship_states, 2)
        x_truth2[2,0] = normalize_angle(x_truth2[2,0])
        x_truth[:num_ownship_states,0] = x_truth0[:num_ownship_states].ravel()
        x_truth[num_ownship_states:2*num_ownship_states,0] = x_truth1[num_ownship_states:2*num_ownship_states].ravel()
        x_truth[2*num_ownship_states:3*num_ownship_states,0] = x_truth2[2*num_ownship_states:3*num_ownship_states].ravel()

        
        ########## BAG TRUTH DATA ##########
        x_truth_bag = np.concatenate((x_truth_bag, x_truth.reshape(-1,1)), axis=1)
        # continue

        ########## ADD NOISE TO TRUTH MOTION ##########
        x_truth_no_noise = deepcopy(x_truth)
        x_truth[0,0] += np.random.normal(0, q)
        x_truth[1,0] += np.random.normal(0, q)
        x_truth[2,0] += np.random.normal(0, q_yaw)
        x_truth[num_ownship_states,0] += + np.random.normal(0, q)
        x_truth[num_ownship_states+1,0] += + np.random.normal(0, q)
        x_truth[num_ownship_states+2,0] += + np.random.normal(0, q_yaw)
        x_truth[2*num_ownship_states,0] += + np.random.normal(0, q)
        x_truth[2*num_ownship_states+1,0] += + np.random.normal(0, q)
        x_truth[2*num_ownship_states+2,0] += + np.random.normal(0, q_yaw)

        ########## PREDICTION STEP  ##########
        asset.predict(u0, Q_perceived)
        asset1.predict(u1, Q_perceived)

        # print(x_truth_no_noise)
        # print(asset.main_filter.x_hat)
        # print(asset1.main_filter.x_hat)
        # print("---")
        # continue #** CHECK

        ########## GENERATE MEASUREMENTS VALUES ##########
        gpsx0 = x_truth[0,0]
        gpsy0 = x_truth[1,0]
        gpsyaw0 = x_truth[2,0]
        gpsx1 = x_truth[num_ownship_states,0]
        gpsy1 = x_truth[num_ownship_states+1,0]
        gpsyaw1 = x_truth[num_ownship_states+2,0]
        gpsx2 = x_truth[2*num_ownship_states,0]
        gpsy2 = x_truth[2*num_ownship_states+1,0]
        gpsyaw2 = x_truth[2*num_ownship_states+2,0]

        # Generate bearing measurements
        src_x = x_truth[0,0]
        src_y = x_truth[1,0]
        src_yaw = x_truth[2,0]
        other_x = x_truth[num_ownship_states,0]
        other_y = x_truth[num_ownship_states+1,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        bearing01 = np.arctan2(diff_y, diff_x) - src_yaw

        # Global 0 bearing
        global_gps_src = np.zeros((2,1))
        src_x = x_truth[0,0]
        src_y = x_truth[1,0]
        src_yaw = x_truth[2,0]
        other_x = global_gps_src[0]
        other_y = global_gps_src[1]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        globalbearing0 = np.arctan2(diff_y, diff_x) - src_yaw

        # Global 1 bearing
        src_x = x_truth[num_ownship_states,0]
        src_y = x_truth[num_ownship_states+1,0]
        src_yaw = x_truth[num_ownship_states+2,0]
        other_x = global_gps_src[0]
        other_y = global_gps_src[1]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        globalbearing1 = np.arctan2(diff_y, diff_x) - src_yaw

        # Relative 01 Range
        src_x = x_truth[0,0]
        src_y = x_truth[1,0]
        other_x = x_truth[num_ownship_states,0]
        other_y = x_truth[num_ownship_states+1,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        range01 = np.sqrt(diff_x**2 + diff_y**2)

        # Global 0 Range
        src_x = x_truth[0,0]
        src_y = x_truth[1,0]
        other_x = global_gps_src[0]
        other_y = global_gps_src[1]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        globalrange0 = np.sqrt(diff_x**2 + diff_y**2)

        # Global 1 Range
        src_x = x_truth[num_ownship_states,0]
        src_y = x_truth[num_ownship_states+1,0]
        other_x = global_gps_src[0]
        other_y = global_gps_src[1]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        globalrange1 = np.sqrt(diff_x**2 + diff_y**2)

        ########## ADD NOIES TO MEASUREMENTS ##########
        gpsx0 += np.random.normal(0, r_gps)
        gpsy0 += np.random.normal(0, r_gps)
        gpsyaw0 += np.random.normal(0, r_gps_yaw)
        gpsx1 += np.random.normal(0, r_gps)
        gpsy1 += np.random.normal(0, r_gps)
        gpsyaw1 += np.random.normal(0, r_gps_yaw)
        gpsx2 += np.random.normal(0, r_gps)
        gpsy2 += np.random.normal(0, r_gps)
        gpsyaw2 += np.random.normal(0, r_gps_yaw)
        gpsyaw2 += np.random.normal(0, r_bearing)
        bearing01 += np.random.normal(0, r_bearing)
        range01 += np.random.normal(0, r_gps)
        globalbearing0 += np.random.normal(0, r_bearing)
        globalrange0 += np.random.normal(0, r_range)
        globalbearing1 += np.random.normal(0, r_bearing)
        globalrange1 += np.random.normal(0, r_range)

        # STOP, CHECK PERFECT MEASUREMENTS
    
        ########## INITIALIZE MEASUREMENT TYPES ##########
        gpsx0_meas = GPSx_Explicit(0, gpsx0, r_gps_perceived, gps_xy_delta)
        gpsy0_meas = GPSy_Explicit(0, gpsy0, r_gps_perceived, gps_xy_delta)
        gpsyaw0_meas = GPSyaw_Explicit(0, gpsyaw0, r_gps_yaw_perceived, gps_yaw_delta)
        gpsx1_meas = GPSx_Explicit(1, gpsx1, r_gps_perceived, gps_xy_delta)
        gpsy1_meas = GPSy_Explicit(1, gpsy1, r_gps_perceived, gps_xy_delta)
        gpsyaw1_meas = GPSyaw_Explicit(1, gpsyaw1, r_gps_yaw_perceived, gps_yaw_delta)
        gpsx2_meas = GPSx_Neighbor_Explicit(0, 2, gpsx2, r_gps_perceived, gps_xy_delta)
        gpsy2_meas = GPSy_Neighbor_Explicit(0, 2, gpsy2, r_gps_perceived, gps_xy_delta)
        gpsyaw2_meas = GPSyaw_Neighbor_Explicit(0,2, gpsyaw2, r_gps_yaw_perceived, gps_yaw_delta)

        bearing01_meas = Azimuth_Explicit(0,1, bearing01, r_bearing_perceived, 0)
        range01_meas = Range_Explicit(0, 1, range01, r_gps_perceived, 0)
        globalbearing0_meas = AzimuthGlobal_Explicit(0, global_gps_src, globalbearing0, r_bearing_perceived, 0)
        globalrange0_meas = RangeGlobal_Explicit(0, global_gps_src, globalrange0, r_range_perceived, 0)
        globalbearing1_meas = AzimuthGlobal_Explicit(1, global_gps_src, globalbearing1, r_bearing_perceived, 0)
        globalrange1_meas = RangeGlobal_Explicit(1, global_gps_src, globalrange1, r_range_perceived, 0)

        ########## ASSETS RECEIVE UNSHAREABLE MEASUREMNTS  ##########
        # asset.receive_meas(gpsx0_meas, shareable=False)
        # asset1.receive_meas(gpsx0_meas, shareable=False)
        # asset1.receive_meas(gpsx1_meas, shareable=False)
        # asset.receive_meas(gpsx2_meas, shareable=False)
        # asset1.receive_meas(gpsx2_measp2, shareable=False)

        # STOP, CHECK Improved estimation of asset by sharing

        ########## ASSETS SHARE MEASUREMENTS  ##########
        sharing = []
        # sharing.append(asset.receive_meas(gpsx0_meas, shareable=True))
        # sharing.append(asset.receive_meas(gpsy0_meas, shareable=True))
        sharing.append(asset.receive_meas(gpsyaw0_meas, shareable=True))
        # sharing.append(asset1.receive_meas(gpsx1_meas, shareable=True))
        # sharing.append(asset1.receive_meas(gpsy1_meas, shareable=True))
        sharing.append(asset1.receive_meas(gpsyaw1_meas, shareable=True))
        sharing.append(asset.receive_meas(gpsx2_meas, shareable=True))
        sharing.append(asset.receive_meas(gpsy2_meas, shareable=True))
        sharing.append(asset.receive_meas(gpsyaw2_meas, shareable=True))
        # Asset 0 passing gps to asset 1
        sharing.append(asset.receive_meas(bearing01_meas, shareable=True))
        sharing.append(asset.receive_meas(range01_meas, shareable=True))
        sharing.append(asset.receive_meas(globalbearing0_meas, shareable=True))
        sharing.append(asset.receive_meas(globalrange0_meas, shareable=True))  
        sharing.append(asset1.receive_meas(globalbearing1_meas, shareable=True))
        sharing.append(asset1.receive_meas(globalrange1_meas, shareable=True))

        # sharing.append(asset.receive_meas(diff_measx, shareable=True))
        # sharing.append(asset.receive_meas(diff_measy, shareable=True))

        # sharing.append(asset1.receive_meas(diff_measx_red, shareable=True))
        # sharing.append(asset1.receive_meas(diff_measy_red, shareable=True))
        
        # Share measurements
        for s in sharing:
            i = s.keys()[0]
            if isinstance(s[i], Implicit):
                implicit_update_cnt += 1
            total_num_meas_cnt += 1
            a = asset_list[i]
            meas = s[i]
            a.receive_shared_meas(meas)

        ########## CORRECTION STEP ##########
        asset.correct()
        asset1.correct()

        ########## RECORD FITLER DATA ##########

        # Estimate 0
        x_hat_bag0 = np.concatenate((x_hat_bag0, asset.main_filter.x_hat), axis=1)
        p_bag0 = np.concatenate((p_bag0, asset.main_filter.P), axis=1)
        x_hat_bag1 = np.concatenate((x_hat_bag1, asset1.main_filter.x_hat), axis=1)
        p_bag1 = np.concatenate((p_bag1, asset1.main_filter.P), axis=1)
            

        ########## DEBUG FILTER INPUTS ##########
        if DEBUG:
            if seq > 37:
                print(x_truth)
                print("meas")
                print(gpsx2)
                print(gpsy2)
                print(gpsyaw2)
                print(asset.main_filter.x_hat)
                print("---")
            if abs(x_truth[2*num_ownship_states+2,0] - asset.main_filter.x_hat[2*num_ownship_states+2,0]) > 1:
                print("Error Large detected")
                break
            # set_trace()

        seq += 1
        print(str(seq) + " out of " + str(K))

        # Plot bagging
        mf0_xhat = deepcopy(asset.main_filter.x_hat)
        mf0_P = deepcopy(asset.main_filter.P)
        mf1_xhat = deepcopy(asset1.main_filter.x_hat)
        mf1_P = deepcopy(asset1.main_filter.P)
        plotting_bag.append([deepcopy(x_truth), mf0_xhat, mf0_P, mf1_xhat, mf1_P])

    # STEP CHECK MEAN ESTIMATES ARE DECENT
    # print(x_truth)
    # print(asset.main_filter.x_hat)
    # print(asset1.main_filter.x_hat)
    # plot_truth_data(x_truth_bag, num_ownship_states)
    # plot_data(plotting_bag, num_ownship_states, num_assets)

    print("Percent of msgs sent implicitly: " + str((implicit_update_cnt / total_num_meas_cnt)*100))
    # PLOT ERROR BOUNDS
    plot_error(x_truth_bag, x_hat_bag0, p_bag0, num_ownship_states, 0)
    plot_error(x_truth_bag, x_hat_bag1, p_bag1, num_ownship_states, 1)

def normalize_angle(angle):
    return np.mod( angle + np.pi, 2*np.pi) - np.pi

if __name__ == "__main__":
    main()