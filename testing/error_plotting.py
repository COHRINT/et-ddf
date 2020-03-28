from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from pdb import set_trace

def get_asset_uncertainty(asset_id, cov):
    first_index_of_asset = asset_id*num_ownship_states
    last_index_of_asset = asset_id*num_ownship_states + 2
    return cov[first_index_of_asset:last_index_of_asset, first_index_of_asset:last_index_of_asset]

def get_plot_labels(num_states, num_ownship_states, asset_id):
    num_assets = int( num_states / num_ownship_states )
    state_correspondence = {}
    for i in range(num_assets):
        if i == asset_id:
            title = "Ownship "
        else:
            title = "of " + str(i) + "'s "
        for j in range(num_ownship_states):
            if num_ownship_states == 2:
                if (j % 2) == 0:
                    final_title = title + "x"
                else:
                    final_title = title + "x_dot"
            elif num_ownship_states == 4:
                if j == 0:
                    final_title = title + "x"
                elif j == 1:
                    final_title = title + "y"
                elif j == 2:
                    final_title = title + "x_dot"
                elif j == 3:
                    final_title = title + "y_dot"
            elif num_ownship_states == 6:
                if j == 0:
                    final_title = title + "x"
                elif j == 1:
                    final_title = title + "y"
                elif j == 2:
                    final_title = title + "yaw"
                elif j == 3:
                    final_title = title + "x_dot"
                elif j == 4:
                    final_title = title + "y_dot"
                elif j == 5:
                    final_title = title + "yaw_dot"
            state_correspondence[(i*num_ownship_states)+j] = final_title
    return state_correspondence

def plot_error(x_truth, x_hat, P, num_ownship_states,asset_id):
    # fig = plt.figure(0)
    # ax = fig.add_subplot(111)
    num_states = x_truth.shape[0]
    print("Generating " + str( num_states ) + " Plots")
    num_assets = int(num_states / num_ownship_states)
    if num_assets == 1 and num_ownship_states == 1:
        truth_data = x_truth[0,:]
        estimate_data = x_hat[0,:]
        error_data = truth_data - estimate_data
        unqqcertainty = P[0, range(0, error_data.size*num_ownship_states, num_ownship_states)]
        error_bound_high = 2*np.sqrt(uncertainty)
        error_bound_low = - 2*np.sqrt(uncertainty)
        ax.plot(error_data, c="r")
        ax.plot(error_bound_high, "--", c="g")
        ax.plot(error_bound_low, "--", c="g")
        ax.set_title("Asset " + str(asset_id) + " Ownship x")
    else:
        title_correspondences = get_plot_labels(num_states, num_ownship_states, asset_id)
        for i in range(num_assets):
            for j in range(num_ownship_states):
                truth_data = x_truth[(i*num_ownship_states)+j,:]
                estimate_data = x_hat[(i*num_ownship_states)+j,:]
                error_data = truth_data - estimate_data
                if "yaw" in title_correspondences[(i*num_ownship_states)+j]:
                    if "dot" not in title_correspondences[(i*num_ownship_states)+j]:
                        error_data = np.mod( error_data + np.pi, 2*np.pi) - np.pi
                uncertainty = P[(i*num_ownship_states)+j, range(i*num_ownship_states+j, error_data.size*num_states, num_states)]
                error_bound_high = 2*np.sqrt(uncertainty)
                error_bound_low = - 2*np.sqrt(uncertainty)
                plt.subplot(num_assets, num_ownship_states, (i*num_ownship_states)+j+1)
                plt.plot(error_data, c="r")
                plt.plot(error_bound_high, "--", c="g")
                plt.plot(error_bound_low, "--", c="g")
                plt.title(title_correspondences[(i*num_ownship_states)+j])
    plt.subplots_adjust(hspace=0.4)
    fig = plt.gcf()
    fig.suptitle(str(asset_id) + "'s estimates", fontsize=14)
    plt.show()
    

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