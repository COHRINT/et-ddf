#!/usr/bin/env python
# -*- encoding: utf-8 -*-
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from scipy.stats import norm as normal
from scipy import integrate
from asset import Asset
from measurements import *
from copy import deepcopy

from pdb import set_trace

np.random.seed(1)

# Simple simulation
K = 20
world_dim = 2
num_assets = 2
num_ownship_states = 6
num_states = num_ownship_states * num_assets

def g_func(x_hat, u, my_id):
    x_state = deepcopy(x_hat) # editing values through function pointers gets strange in Python
    print("Input for asset " + str(my_id) + ":")
    print(x_state)
    print(u)
    u_input = deepcopy(u)
    x_state[my_id * num_ownship_states + 3] = u_input[0,0] # speed
    x_state[my_id * num_ownship_states + 5] = u_input[1,0] # angular velocity

    G = np.zeros((num_states, num_states))
    for a in range(num_assets):
        start_index = a*num_ownship_states
        s = x_state[start_index + 3,0]
        theta_dot = x_state[start_index + 5,0]

        theta_initial = x_state[start_index+2,0]
        def dynamics(t, z):
            _x_dot = s * np.cos(z[2])
            _y_dot = s * np.sin(z[2])
            _theta_dot = theta_dot
            return np.array([_x_dot, _y_dot, _theta_dot])

        t_init, t_final = 0, 1
        z_init = x_state[start_index:start_index + 3,0]
        r = integrate.RK45(dynamics, t_init, z_init, t_final)
        while r.status == "running":
            status = r.step()

        x_state[start_index: start_index+3,0] = r.y
        
        # Construct this asset's part of jacobian
        G[start_index,start_index] = 1
        G[start_index + 1,start_index + 1] = 1
        G[start_index + 2, start_index + 2] = 1
        G[start_index, start_index + 2] = -s * np.sin(theta_initial + theta_dot/2)
        G[start_index + 1, start_index + 2] = s * np.cos(theta_initial + theta_dot/2)
        # G[start_index, start_index + 2] = -s * np.sin(theta_initial)
        # G[start_index + 1, start_index + 2] = s * np.cos(theta_initial)
        G[start_index + 4, start_index + 4] = 1

        if a != my_id:
            G[start_index + 3, start_index + 3] = 1
            G[start_index + 5, start_index + 5] = 1

    print("Output")
    print(x_state)
    return (x_state, G)
    
def createQ(q,q_yaw,q_xy_guess,q_yaw_guess, asset_id):
    Q = np.zeros((num_states,num_states))
    for i in range(num_assets):
        if i == asset_id:
            Q[i*num_ownship_states,i*num_ownship_states] = q ** 2
            Q[i*num_ownship_states+1,i*num_ownship_states+1] = q ** 2
            Q[i*num_ownship_states+2,i*num_ownship_states+2] = q_yaw ** 2
        else:
            Q[i*num_ownship_states,i*num_ownship_states] = q_xy_guess ** 2
            Q[i*num_ownship_states+1,i*num_ownship_states+1] = q_xy_guess ** 2
            Q[i*num_ownship_states+2,i*num_ownship_states+2] = q_yaw_guess ** 2
    return deepcopy(Q)

def main():

    # A = np.eye(num_states)
    # B = np.eye(num_states)

    # Process noise
    q = 0.1
    q_yaw = 0.01

    q_xy_guess = 2
    q_yaw_guess = np.pi / 5

    # Meas noise
    r = 0.5
    R = np.eye(num_states) * (r**2)
    r_yaw = 0.1

    # (0,0,0)
    x_truth = np.zeros((num_states,1))

    # Asset 2 (0,15.6,-pi)
    x_truth[num_ownship_states+1, 0] = 15.6
    x_truth[num_ownship_states+2, 0] = -np.pi

    # Asset 3 (5,0,pi/2)
    # x_truth[2*num_ownship_states, 0] = 5
    # x_truth[2*num_ownship_states+2,0] = np.pi/2

    initial_P = np.zeros((num_states,num_states))     
    # Set yaw states to start at a valid range
    if world_dim == 2 and num_ownship_states in [3,6]:
        for i in range(num_assets):
            asset_yaw_index = i*num_ownship_states + 2
            x_truth[asset_yaw_index,0] = normalize_angle(x_truth[asset_yaw_index,0])
    elif world_dim == 3:
        # Assume x,y,z,roll,pitch,yaw, x_dot along base_link, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot
        for i in range(num_assets):
            asset_roll_index = i*num_ownship_states + 3
            asset_pitch_index = i*num_ownship_states + 4
            asset_yaw_index = i*num_ownship_states + 5
            x_truth[asset_roll_index] = normalize_angle(x_truth[asset_roll_index])
            x_truth[asset_pitch_index] = normalize_angle(x_truth[asset_pitch_index])
            x_truth[asset_yaw_index] = normalize_angle(x_truth[asset_yaw_index])
            x_truth[yaw_index,0] = np.random.uniform(-np.pi, np.pi)


    delta_gps_xy = 3
    delta_gps_yaw = 0.3
    delta_linrel = 3

    implicit_update_cnt = 0
    total_num_meas_cnt = 0

    asset_list = []
    asset = Asset(0, num_ownship_states, world_dim, x_truth, initial_P, g_func, red_team=[2])
    asset1 = Asset(1, num_ownship_states, world_dim, x_truth, initial_P, g_func, red_team=[2])
    # asset = Asset(0, world_dim, x_truth, initial_P, A, B, delta, red_team=[2])
    # asset1 = Asset(1, world_dim, x_truth, initial_P, A, B, delta, red_team=[2])
    asset_list.append(asset)
    asset_list.append(asset1)

    bag = []
    x_truth_bag = np.array([])
    x_hat_bag0 = np.array([])
    x_hat_bag1 = np.array([])
    p_bag0 = np.array([])
    p_bag1 = np.array([])
    
    x_estimate_bag = np.array([])
    seq = 0

    for k in range(K):
        speed = 1
        theta_dot = np.pi / 20
        u = np.array([[speed],[theta_dot]]) #np.random.randint(-1, 1, size=(num_states,1))
        u_red = np.array([[0.15],[0]])
        # w = np.random.normal(0, np.sqrt(q), size=(num_states,1))

        # Process Noisee
        w1 = np.zeros((num_states,1))
        for i in range(num_assets):
            w1[i*num_ownship_states,0] = np.random.normal(0, q)
            w1[i*num_ownship_states+1,0] = np.random.normal(0, q)
            w1[i*num_ownship_states+2,0] = np.random.normal(0, q_yaw)
        
        # Truth update
        print("updating truth 0")
        x_truth_0, G = g_func(x_truth, u, 0)
        print("updating truth 1")
        x_truth_1, G = g_func(x_truth, u, 1)
        # x_truth_2, G = g_func(x_truth, u_red, 2)

        x_truth[:num_ownship_states,0] = x_truth_0[:num_ownship_states,0]
        x_truth[num_ownship_states:2*num_ownship_states,0] = x_truth_1[num_ownship_states:2*num_ownship_states,0]
        # x_truth[2*num_ownship_states:, 0] = x_truth_2[2*num_ownship_states:,0]
        x_truth += w1

        for i in range(num_assets):
            x_truth[i*num_ownship_states + 2,0] = normalize_angle(x_truth[i*num_ownship_states+2,0])
        
        gps_meas = x_truth + np.random.multivariate_normal( np.zeros((num_states,)), R).reshape(-1,1)

        # relative_dist_01 = x_truth[3:5,0] - x_truth[:2,0] + np.random.multivariate_normal( np.zeros((world_dim,)), np.eye(world_dim)*(r**2)))
        # diff_measx = LinRelx_Explicit(0, 1, relative_dist_01[0], r**2, delta_linrel)
        # diff_measy = LinRely_Explicit(0, 1, relative_dist_01[1], r**2, delta_linrel)

        # relative_dist_12_x = x_truth[2*num_ownship_states,0] - x_truth[num_ownship_states,0] + np.random.normal(0,r)
        # relative_dist_12_y = x_truth[2*num_ownship_states+1,0] - x_truth[num_ownship_states+1,0] + np.random.normal(0,r)
        # diff_measx_red = LinRelx_Explicit(1, 2, relative_dist_12_x, r, delta_linrel)
        # diff_measy_red = LinRely_Explicit(1, 2, relative_dist_12_y, r, delta_linrel)

        x_meas0 = gps_meas[0,0]
        y_meas0 = gps_meas[1,0]
        yaw_meas0 = x_truth[2,0] + np.random.normal(0, r_yaw)
        x_meas1 = gps_meas[num_ownship_states,0]
        y_meas1 = gps_meas[num_ownship_states+1,0]
        yaw_meas1 = x_truth[num_ownship_states+2,0] + np.random.normal(0, r_yaw)

        q = 0.5; q_yaw = 0.2; q_xy_guess = 1; q_yaw_guess=0.1
        Q0 = createQ(q, q_yaw, q_xy_guess, q_yaw_guess, 0)
        Q1 = createQ(q, q_yaw, q_xy_guess, q_yaw_guess, 1)
        # print("asset 0 predicting")
        # print(u)
        # print(Q0)
        asset.predict(u, Q0)
        # print("asset 1 predicting")
        # print(u)
        # print(Q1)
        asset1.predict(u, Q1)
        # print("truth:")
        # print(x_truth)
        # print("prediction0:")
        # print(asset.main_filter.x_hat)
        # print(asset.main_filter.P)
        # print("prediction1:")
        # print(asset1.main_filter.x_hat)
        # print(asset1.main_filter.P)
        # set_trace()

        print(x_meas0)
        print(y_meas0)
        print(yaw_meas0)

        gpsx0 = GPSx_Explicit(0, x_meas0, r**2, delta_gps_xy)
        gpsy0 = GPSy_Explicit(0, y_meas0, r**2, delta_gps_xy)
        gpsyaw0 = GPSyaw_Explicit(0, yaw_meas0, r_yaw**2, delta_gps_yaw)
        gpsx1 = GPSx_Explicit(1, x_meas1, r**2, delta_gps_xy)
        gpsy1 = GPSy_Explicit(1, y_meas1, r**2, delta_gps_xy)
        gpsyaw1 = GPSyaw_Explicit(1, yaw_meas1, r_yaw**2, delta_gps_yaw)

        # asset.receive_meas(gpsx0, shareable=False)
        # asset.receive_meas(gpsy0, shareable=False)
        # asset.receive_meas(gpsyaw0, shareable=False)
        # asset1.receive_meas(gpsx1, shareable=False)
        # asset1.receive_meas(gpsy1, shareable=False)
        # asset1.receive_meas(gpsyaw1, shareable=False)

        sharing = []
        sharing.append(asset.receive_meas(gpsx0, shareable=True))
        sharing.append(asset.receive_meas(gpsy0, shareable=True))
        sharing.append(asset.receive_meas(gpsyaw0, shareable=True))
        sharing.append(asset1.receive_meas(gpsx1, shareable=True))
        sharing.append(asset1.receive_meas(gpsy1, shareable=True))
        sharing.append(asset1.receive_meas(gpsyaw1, shareable=True))

        # sharing.append(asset.receive_meas(diff_measx, shareable=True))
        # sharing.append(asset.receive_meas(diff_measy, shareable=True))

        # sharing.append(asset1.receive_meas(diff_measx_red, shareable=True))
        # sharing.append(asset1.receive_meas(diff_measy_red, shareable=True))
        

        for s in sharing:
            i = s.keys()[0]
            if isinstance(s[i], Implicit):
                implicit_update_cnt += 1
            total_num_meas_cnt += 2
            a = asset_list[i]
            meas = s[i]
            # print("Asset "+ str(meas.src_id) + " sharing with " + str(i) + " " + s[i].__class__.__name__ + "| data: " + str(meas.data))
            a.receive_shared_meas(meas)

        asset.correct()
        asset1.correct()

        # if x_estimate_bag.size == 0:
        #     x_estimate_bag = asset.main_filter.x_hat
        # else:
        #     x_estimate_bag = np.concatenate((x_estimate_bag, asset.main_filter.x_hat), axis=1)
        
        # Truth
        if x_truth_bag.size == 0:
            x_truth_bag = deepcopy(x_truth)
        else:
            x_truth_bag = np.concatenate((x_truth_bag, x_truth), axis=1)

        # Estimate 0
        if x_hat_bag0.size == 0:
            x_hat_bag0 = deepcopy(asset.main_filter.x_hat)
        else:
            x_hat_bag0 = np.concatenate((x_hat_bag0, asset.main_filter.x_hat), axis=1)

        # Estimate 1
        if x_hat_bag1.size == 0:
            x_hat_bag1 = deepcopy(asset1.main_filter.x_hat)
        else:
            x_hat_bag1 = np.concatenate((x_hat_bag1, asset1.main_filter.x_hat), axis=1)

        # Uncertainty 0
        if p_bag0.size == 0:
            p_bag0 = deepcopy(asset.main_filter.P)
        else:
            p_bag0 = np.concatenate((p_bag0, asset.main_filter.P), axis=1)

        # Uncertainty 1
        if p_bag1.size == 0:
            p_bag1 = deepcopy(asset1.main_filter.P)
        else:
            p_bag1 = np.concatenate((p_bag1, asset1.main_filter.P), axis=1)

        bag.append([deepcopy(x_truth), deepcopy(asset.main_filter.x_hat), deepcopy(asset.main_filter.P)])

        seq += 1
        print(seq)
        # print(x_truth)
        # print("Estimate0: w/ sd")
        # print(asset.main_filter.x_hat)
        # print(np.matrix.round( 2*np.sqrt(asset.main_filter.P), 3))
        # print("Estimate1 w/ sd:")
        # print(asset1.main_filter.x_hat)
        # print(np.matrix.round( 2*np.sqrt(asset1.main_filter.P), 3))
        # print(asset.main_filter.P)
        # set_trace()
    plot_error(x_truth_bag, x_hat_bag0, p_bag0, 0)
    plot_error(x_truth_bag, x_hat_bag1, p_bag1, 1)

    # plot_truth_data(x_truth_bag)
    # print("########################")
    # plot_data(bag, world_dim)
    # asset_ids = np.linspace(0, num_assets-1, num_assets).reshape(-1,1)
    # z = (np.zeros((num_assets, num_ownship_states)) + asset_ids).reshape(-1,1)
    # print("Truth:")
    # print(np.concatenate((z,np.matrix.round(x_truth,2)), axis=1))
    # asset.print_filters(main_only=True, mean_only=True, cov_sd_form=False)
    # asset1.print_filters(main_only=True, mean_only=True, cov_sd_form=False)

    # asset0.print_filters()
    # asset1.print_filters()
    # print("---------------------------------------------------------------")
    # print("x_hat: \n" + str(asset0.main_filter.x_hat))
    # print("P: \n" + str(asset0.main_filter.P))
    # print("Uncertainty sigma: \n" + str(2*np.sqrt(asset0.main_filter.P)))

    # uncertainty_range = [x_hat - 2*np.sqrt(P), x_hat + 2*np.sqrt(P)]
    # print("Uncertainty range: " + str(uncertainty_range))
    # print("Inside range? " + str(x > uncertainty_range[0] and x < uncertainty_range[1]))

    # print("Num implicit percentage: " + str(implicit_update_cnt / (6*K) ))

def plot_error(x_truth, x_hat, P, asset_id):
    fig = plt.figure(0)
    ax = fig.add_subplot(111)
    # fig, ax = plt.subplots(111)
    # fig, ax = plt.subplots(num_assets, num_ownship_states)
    i = j = 0
    truth_data = x_truth[i+j,:]
    estimate_data = x_hat[i+j,:]
    # k = np.linspace(0,1,estimate_data.size).reshape(-1,1)
    error_data = truth_data - estimate_data
    uncertainty = P[i+j, range(i+j, error_data.size*num_ownship_states, num_ownship_states)]
    error_bound_high = 2*np.sqrt(uncertainty)
    error_bound_low = - 2*np.sqrt(uncertainty)
    print(error_data)
    print(error_data.shape)
    plt.plot(error_data, c="r")
    plt.plot(error_bound_high, "--", c="g")
    plt.plot(error_bound_low, "--", c="g")
    # ax[0].plot(error_bound_high, "--", c="b")
    # ax[0].plot(error_bound_low, "--", c="b")
    plt.show()
    # for i in range(num_assets):
    #     for j in range(num_ownship_states):
            

def normalize_angle(angle):
    return np.mod( angle + np.pi, 2*np.pi) - np.pi

def get_asset_uncertainty(asset_id, cov):
    first_index_of_asset = asset_id*num_ownship_states
    last_index_of_asset = asset_id*num_ownship_states + 2
    return cov[first_index_of_asset:last_index_of_asset, first_index_of_asset:last_index_of_asset]

def plot_truth_data(bag):
    fig = plt.figure(0)
    ax = fig.add_subplot(111)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    ax.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    ax.scatter(bag[0,:], bag[1,:], c="k", s=20)
    ax.scatter(bag[num_ownship_states,:], bag[num_ownship_states + 1,:], c="b", s=20)
    ax.scatter(bag[2*num_ownship_states,:], bag[2*num_ownship_states + 1,:], c="r", s=20)
    plt.xlabel("x", fontsize=10)
    plt.ylabel("y", fontsize=10)
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)
    # ax.legend(fontsize=12)

    plt.show()
    # plt.close()

def plot_data(bag, world_dim):
    num_points = 500
    pts = np.linspace(0, 2*np.pi, num_points)
    x_pts = 2*np.cos(pts)
    y_pts = 2*np.sin(pts)
    circle_pts = np.append(x_pts, y_pts).reshape((2,-1))
    # num_assets = int( bag[0][0].size / world_dim )

    seq = 0
    for [x_truth, mean, cov] in bag:
        fig = plt.figure(0)
        ax = fig.add_subplot(111)
        ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))

        # print(x_truth)
        # print(mean)
        # print(cov)
        color_array = ["b","g","r"]
        for i in range(num_assets):
            ax.scatter(x_truth[i*num_ownship_states,0], x_truth[i*num_ownship_states+1,0], c=color_array[i], label="truth", s=20, marker="s")
            mle = mean[i*num_ownship_states:i*num_ownship_states+2,0].reshape(-1,1)
            # ax.scatter(mle[0,0], mle[1,0], c="b", label="estimate", s=4)
            
            asset_uncertainty = np.matrix.round( get_asset_uncertainty(i, cov), 2)
            
            cov_pts = np.dot(2*np.sqrt(asset_uncertainty), circle_pts) + mle
            ax.scatter(cov_pts[0,:], cov_pts[1,:], c=color_array[i], label="estimate", s=4)

        lim = 10
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        title = "plt_" + str(seq)
        plt.title(title, fontsize=18)
        plt.xlabel("x", fontsize=10)
        plt.ylabel("y", fontsize=10)
        plt.xticks(fontsize=10)
        plt.yticks(fontsize=10)
        # ax.legend(fontsize=12)

        # plt.show()
        # plt.close()

        plt.savefig("recording/" + title)
        plt.clf()
        seq += 1
        print(seq / len(bag))

if __name__ == "__main__":
    main()