#!/usr/bin/env python
from __future__ import division
import rosbag
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
import tf

ASSET_NAME = "bluerov2_4"

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
                    final_title = title + "z"
                elif j == 3:
                    final_title = title + "x_dot"
                elif j == 4:
                    final_title = title + "y_dot"
                elif j == 5:
                    final_title = title + "z_dot"
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

def convert_numpy(cov_msg):
    x = np.zeros((6,1))
    P = np.zeros((6,6))
    x[0,0] = cov_msg.pose.pose.position.x
    x[1,0] = cov_msg.pose.pose.position.y
    x[2,0] = cov_msg.pose.pose.position.z
    x[3,0] = cov_msg.twist.twist.linear.x
    x[4,0] = cov_msg.twist.twist.linear.y
    x[5,0] = cov_msg.twist.twist.linear.z
    tmp = np.array(cov_msg.pose.covariance).reshape(6,6)
    P[:3,:3] = tmp[:3,:3]
    tmp = np.array(cov_msg.twist.covariance).reshape(6,6)
    P[3:,3:] = tmp[:3,:3]
    return x,P
def convert_numpy8(cov_msg):
    x = np.zeros((8,1))
    P = np.zeros((8,8))
    x[0,0] = cov_msg.pose.pose.position.x
    x[1,0] = cov_msg.pose.pose.position.y
    x[2,0] = cov_msg.pose.pose.position.z
    _, _, yaw = tf.transformations.euler_from_quaternion([cov_msg.pose.pose.orientation.x, \
                                                        cov_msg.pose.pose.orientation.y, \
                                                        cov_msg.pose.pose.orientation.z, \
                                                        cov_msg.pose.pose.orientation.w])
    x[3,0] = yaw # * (180 / np.pi)
    x[4,0] = cov_msg.twist.twist.linear.x
    x[5,0] = cov_msg.twist.twist.linear.y
    x[6,0] = cov_msg.twist.twist.linear.z
    x[7,0] = cov_msg.twist.twist.angular.z
    tmp = np.array(cov_msg.pose.covariance).reshape(6,6)
    P[:3,:3] = tmp[:3,:3]
    P[3,3] = tmp[5,5]
    tmp = np.array(cov_msg.twist.covariance).reshape(6,6)
    P[4:7,4:7] = tmp[:3,:3]
    P[7,7] = tmp[5,5]
    return x,P

parser = argparse.ArgumentParser(description='Bag Analysis Node')
parser.add_argument("-b", "--bag", type=str, help="Bag File to Analyze", required=True)
parser.add_argument("-e", "--etddf", type=bool, help="Analyze ETDDF (True) or Strapdown (False)",default=False, required=False)
args = parser.parse_args()


bag = rosbag.Bag(args.bag)
synced_3of3 = []
synced_times = []
last_pose_gt = None
last_pose_gt_time = None

if args.etddf == True:
    etddf_topic = '/'+ASSET_NAME + '/etddf/estimate/' + ASSET_NAME
else:
    etddf_topic = '/'+ASSET_NAME + '/strapdown/estimate'
# etddf_topic = '/bluerov2_4/odometry/filtered'
for topic, msg, t in bag.read_messages(topics=[etddf_topic, '/' + ASSET_NAME+ '/pose_gt']):
    if topic == etddf_topic and last_pose_gt is not None:
        synced_3of3.append([msg, last_pose_gt])
        synced_times.append([t, last_pose_gt_time])
    elif topic == etddf_topic:
        continue
    else:
        last_pose_gt = msg
        last_pose_gt_time = t

# modem_cnt = 0
# for topic, msg, t in bag.read_messages(topics=['/bluerov2_4/etddf/packages_in']):
#     asset = msg.measurements[0].measured_asset
#     if asset == "bluerov2_4":
#         modem_cnt += 1
# print(modem_cnt)
bag.close()

x_truth_bag = None
x_hat_bag0 = None
p_bag0 = None

error = []
for [estimate, pose_gt] in synced_3of3:
    x_hat, P = convert_numpy8(estimate)
    # x_hat, P = convert_enu(x_hat, P)
    x_truth, _ = convert_numpy8(pose_gt)
    if x_truth_bag is None:
        x_truth_bag = x_truth
        x_hat_bag0 = x_hat
        p_bag0 = P
    else:
        x_hat_bag0 = np.concatenate((x_hat_bag0, x_hat), axis=1)
        p_bag0 = np.concatenate((p_bag0, P), axis=1)
        x_truth_bag = np.concatenate((x_truth_bag, x_truth), axis=1)

print(x_hat_bag0.shape)
print(p_bag0.shape)
print(x_truth_bag.shape)

plot_error(x_truth_bag, x_hat_bag0, p_bag0, 8,0)
# Get erros
# Plot errors (x,y,z) && covariances
# Overlay times modem measurements were received (red lines)

# Make filter conservative, add in reseting of IMU filter
# Move assets and tune process noise
# Verify meas pkgs getting sent and received
# Test with 360 degree sonar, compare error plots w/ and without sonar
# Create video, everything works and we're done