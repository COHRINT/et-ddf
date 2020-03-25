#!/usr/bin/env python
# -*- encoding: utf-8 -*-
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from scipy.stats import norm as normal
from asset import Asset
from measurements import *

def main():
    # Simple simulation
    K = 10
    dim = 2
    num_robots = 2
    num_states = dim * num_robots
    A = np.eye(num_states)
    B = np.eye(num_states)

    # Motion noise
    q = 2
    Q = np.eye(num_states) * (q ** 2)
    r = 1
    R = np.eye(num_states) * (r ** 2)

    # Estimates
    x_truth = np.random.randint(-5,5, size=(num_states,1))
    initial_P = np.zeros((num_states,num_states)) 

    delta = 5

    implicit_update_cnt = 0

    asset_list = []
    asset = Asset(0, dim, x_truth, initial_P, A, B, delta)
    asset1 = Asset(1, dim, x_truth, initial_P, A, B, delta)
    asset_list.append(asset)
    asset_list.append(asset1)

    bag = []

    for k in range(K):
        u = np.random.randint(-1, 1, size=(num_states,1))
        w = np.random.normal(0, q, size=(num_states,1))

        # Truth update
        x_truth = A.dot(x_truth) + B.dot(u) + w
        meas = x_truth + np.random.multivariate_normal( np.zeros((num_states,)), R).reshape(-1,1)
        relative_dist_01 = x_truth[2:,0] - x_truth[:2,0] + np.random.multivariate_normal( np.zeros((dim,)), np.eye(dim)*(r**2))
        diff_measx = LinRelx_Explicit(0, 1, relative_dist_01[0], r)
        diff_measy = LinRely_Explicit(0, 1, relative_dist_01[1], r)

        x_meas0 = meas[0,0]
        y_meas0 = meas[1,0]
        x_meas1 = meas[2,0]
        y_meas1 = meas[3,0]

        asset.predict(u,Q)
        asset1.predict(u,Q)

        gpsx0 = GPSx_Explicit(0, x_meas0, r)
        gpsy0 = GPSy_Explicit(0, y_meas0, r)
        gpsx1 = GPSx_Explicit(1, x_meas1, r)
        gpsy1 = GPSy_Explicit(1, y_meas1, r)

        sharing = []
        sharing.append(asset.receive_meas(gpsx0, shareable=True))
        sharing.append(asset.receive_meas(gpsy0, shareable=True))
        sharing.append(asset.receive_meas(diff_measx, shareable=True))
        sharing.append(asset.receive_meas(diff_measy, shareable=True))
        # sharing.append(asset1.receive_meas(gpsx1, shareable=True))
        # sharing.append(asset1.receive_meas(gpsy1, shareable=True))
        

        for s in sharing:
            i = s.keys()[0]
            # print("Asset "+str(i^1) + " sharing with " + str(i) + " " + s[i].__class__.__name__)
            if isinstance(s[i], Implicit):
                implicit_update_cnt += 1
            a = asset_list[i]
            a.receive_shared_meas(s[i])

        asset.correct()
        asset1.correct()

        bag.append([x_truth, asset.main_filter.x_hat, asset.main_filter.P])

        # shared_meas2 = 
        # if isinstance(shared_meas2[0], Implicit):
            # implicit_update_cnt += 1
        # asset0.receive_shared_meas(1, shared_meas2[0])

        # By sharing measurements, asset0's position becomes fully observable
        # asset0.correct()
        # asset1.correct()

    plot_data(bag, dim)

    print("Truth: \n" + str(x_truth))

    asset.print_filters(main_only=True)
    asset1.print_filters(main_only=True)
    # asset1.print_filters(main_only=True)
    # asset0.print_filters()
    # asset1.print_filters()
    print("---------------------------------------------------------------")
    # print("x_hat: \n" + str(asset0.main_filter.x_hat))
    # print("P: \n" + str(asset0.main_filter.P))
    # print("Uncertainty sigma: \n" + str(2*np.sqrt(asset0.main_filter.P)))

    # uncertainty_range = [x_hat - 2*np.sqrt(P), x_hat + 2*np.sqrt(P)]
    # print("Uncertainty range: " + str(uncertainty_range))
    # print("Inside range? " + str(x > uncertainty_range[0] and x < uncertainty_range[1]))
    print("Num implicit percentage: " + str(implicit_update_cnt / (4*K) ))

def get_asset_uncertainty(asset_id, cov, dim):
    first_index_of_asset = asset_id*dim
    last_index_of_asset = asset_id*dim + dim
    return cov[first_index_of_asset:last_index_of_asset, first_index_of_asset:last_index_of_asset]

def plot_data(bag, dim):
    num_points = 500
    pts = np.linspace(0, 2*np.pi, num_points)
    x_pts = 2*np.cos(pts)
    y_pts = 2*np.sin(pts)
    circle_pts = np.append(x_pts, y_pts).reshape((2,-1))
    num_assets = int( bag[0][0].size / dim )

    seq = 0
    for [x_truth, mean, cov] in bag:
        fig = plt.figure(0)
        ax = fig.add_subplot(111)
        ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))

        color_array = ["b","g","r"]
        for i in range(num_assets):
            ax.scatter(x_truth[i*dim,0], x_truth[i*dim+1,0], c=color_array[i], label="truth", s=20, marker="s")
            mle = mean[i*dim:i*dim+2,0].reshape(-1,1)
            # ax.scatter(mle[0,0], mle[1,0], c="b", label="estimate", s=4)
            
            asset_uncertainty = get_asset_uncertainty(i, cov, dim)
            cov_pts = np.dot(2*np.sqrt(asset_uncertainty), circle_pts) + mle
            ax.scatter(cov_pts[0,:], cov_pts[1,:], c=color_array[i], label="estimate", s=4)

        lim = 20
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        title = "plt_" + str(seq)
        plt.title(title, fontsize=18)
        plt.xlabel("x", fontsize=10)
        plt.ylabel("y", fontsize=10)
        plt.xticks(fontsize=10)
        plt.yticks(fontsize=10)
        # ax.legend(fontsize=12)

        plt.show()
        plt.close()

        # plt.savefig("recording/" + title)
        # plt.clf()
        seq += 1

if __name__ == "__main__":
    main()