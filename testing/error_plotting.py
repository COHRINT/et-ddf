from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
from pdb import set_trace

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
            elif num_ownship_states == 8:
                if j == 0:
                    final_title = title + "x"
                elif j == 1:
                    final_title = title + "y"
                elif j == 2:
                    final_title = title + "z"
                elif j == 3:
                    final_title = title + "yaw"
                elif j == 4:
                    final_title = title + "x_dot"
                elif j == 5:
                    final_title = title + "y_dot"
                elif j == 6:
                    final_title = title + "z_dot"
                elif j == 7:
                    final_title = title + "yaw_dot"
            state_correspondence[(i*num_ownship_states)+j] = final_title
    return state_correspondence

def plot_error(x_truth, x_hat, P, num_ownship_states,asset_id):
    num_states = x_truth.shape[0]
    print("Generating " + str( num_states ) + " Plots")
    num_assets = int(num_states / num_ownship_states)
    if num_assets == 1 and num_ownship_states == 1:
        truth_data = x_truth[0,:]
        estimate_data = x_hat[0,:]
        error_data = truth_data - estimate_data
        uncertainty = P[0, range(0, error_data.size*num_ownship_states, num_ownship_states)]
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
    

def plot_truth_data(bag, num_ownship_states):
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

def plot_data(bag, num_ownship_states, num_assets, global_gps_src):

    # TODO Was last used for 2D, may need to be updated for 3D

    num_points = 500
    pts = np.linspace(0, 2*np.pi, num_points)
    x_pts = 2*np.cos(pts)
    y_pts = 2*np.sin(pts)
    circle_pts = np.append(x_pts, y_pts).reshape((2,-1))
    # num_assets = int( bag[0][0].size / world_dim )
    orientation_len = 2
    x_lim_pos = 17
    x_lim_neg = -17
    y_lim_pos = 12
    y_lim_neg = -12
    seq = 0
    for [x_truth, x_hat0, P0, x_hat1, P1] in bag:
        #### ASSET 0 ####
        ax = plt.subplot(1, 2, 1, aspect="equal", adjustable="box-forced")
        ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax.scatter(global_gps_src[0,0], global_gps_src[1,0], c="k", label="truth", s=20, marker="s")

        penetration_rect = mpatches.Rectangle((-10,-5), 20,10, ls="--", color="black", fill=False)
        ax.add_patch(penetration_rect)

        color_array = ["b","g","r"]
        for i in range(num_assets):
            if i == 2:
                ax.scatter(x_truth[i*num_ownship_states,0], x_truth[i*num_ownship_states+1,0], c="r", label="truth", s=10, marker="s")
            else:
                ax.scatter(x_truth[i*num_ownship_states,0], x_truth[i*num_ownship_states+1,0], c="k", label="truth", s=10, marker="s")
            mle = x_hat0[i*num_ownship_states:i*num_ownship_states+2,0].reshape(-1,1)
                    
            cov_asset = np.matrix.round( P0[i*num_ownship_states:i*num_ownship_states+2, i*num_ownship_states:i*num_ownship_states+2], 2)
            cov_asset = 2*np.sqrt(np.abs(cov_asset)) * np.sign(cov_asset)
            cov_pts = np.dot(cov_asset, circle_pts) + mle
            ax.scatter(cov_pts[0,:], cov_pts[1,:], c=color_array[i], label="estimate", s=4)

            ori = x_hat0[i*num_ownship_states + 2, 0]
            pt = orientation_len * np.array([[np.cos(ori)],[np.sin(ori)]]) + mle
            plt.plot([mle[0],pt[0]], [mle[1],pt[1]], lw=2, c="k")

            # Plot Sonar Scan range
            patches = []
            if i == 0:
                ori_deg = x_truth[i*num_ownship_states+2,0] * (180/np.pi)
                wedge = mpatches.Wedge( (x_truth[i*num_ownship_states:i*num_ownship_states+2,0]), 10, -65 + ori_deg, 65 + ori_deg, ec="none")
                patches.append(wedge)
                colors=np.linspace(0,1, len(patches))
                collection = PatchCollection(patches, cmap=plt.cm.hsv, alpha=0.3)
                collection.set_array(np.array(colors))
                ax.add_collection(collection)

        ax.set_xlim(x_lim_neg, x_lim_pos)
        ax.set_ylim(y_lim_neg, y_lim_pos)
        plt.xlabel("x", fontsize=10)
        plt.ylabel("y", fontsize=10)
        plt.xticks(fontsize=10)
        plt.yticks(fontsize=10)
        plt.title("Asset " + str(0) + "'s Estimates")

        #### ASSET 1 ####
        ax1 = plt.subplot(1, 2, 2, aspect="equal", adjustable="box-forced",sharex=ax, sharey=ax)
        ax1.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax1.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        ax1.scatter(global_gps_src[0,0], global_gps_src[1,0], c="k", label="truth", s=20, marker="s")
        penetration_rect = mpatches.Rectangle((-10,-5), 20,10, ls="--", color="black", fill=False)
        ax1.add_patch(penetration_rect)

        color_array = ["b","g","r"]
        for i in range(num_assets):
            if i == 2:
                ax1.scatter(x_truth[i*num_ownship_states,0], x_truth[i*num_ownship_states+1,0], c="r", label="truth", s=10, marker="s")
            else:
                ax1.scatter(x_truth[i*num_ownship_states,0], x_truth[i*num_ownship_states+1,0], c="k", label="truth", s=10, marker="s")
            mle = x_hat1[i*num_ownship_states:i*num_ownship_states+2,0].reshape(-1,1)
                    
            cov_asset = np.matrix.round( P1[i*num_ownship_states:i*num_ownship_states+2, i*num_ownship_states:i*num_ownship_states+2], 2)
            cov_asset = 2*np.sqrt(np.abs(cov_asset)) * np.sign(cov_asset)
            cov_pts = np.dot(cov_asset, circle_pts) + mle
            ax1.scatter(cov_pts[0,:], cov_pts[1,:], c=color_array[i], label="estimate", s=4)

            ori = x_hat0[i*num_ownship_states + 2, 0]
            pt = orientation_len * np.array([[np.cos(ori)],[np.sin(ori)]]) + mle
            plt.plot([mle[0],pt[0]], [mle[1],pt[1]], lw=2, c="k")

            # Plot Sonar Scan range
            patches = []
            if i == 1:
                ori_deg = x_truth[i*num_ownship_states+2,0] * (180/np.pi)
                wedge = mpatches.Wedge( (x_truth[i*num_ownship_states:i*num_ownship_states+2,0]), 10, -65 + ori_deg, 65 + ori_deg, ec="none")
                patches.append(wedge)
                colors=np.linspace(0,1, len(patches))
                collection = PatchCollection(patches, cmap=plt.cm.hsv, alpha=0.3)
                collection.set_array(np.array(colors))
                ax1.add_collection(collection)


        ax.set_xlim(x_lim_neg, x_lim_pos)
        ax.set_ylim(y_lim_neg, y_lim_pos)
        plt.xlabel("x", fontsize=10)
        plt.xticks(fontsize=10)
        plt.yticks(fontsize=10)
        plt.title("Asset " + str(1) + "'s Estimates")
        # plt.setp(ax1.get_xticklabels(), visible=False)
        plt.setp(ax1.get_yticklabels(), visible=False)

        fig = plt.gcf()
        title = "t = " + str(seq)
        fig.suptitle(title, fontsize=18)
        # ax.legend(fontsize=12)

        mng = plt.get_current_fig_manager()
        mng.full_screen_toggle()
        # plt.show()
        # plt.close()

        plt.savefig("recording/plt_" + str(seq))
        plt.clf()
        seq += 1
        print(round((seq / len(bag))*100,1))