#!/usr/bin/env python
from __future__ import division
"""@package etddf

ROS interface script for delta tiering filter

"""

from etddf.delta_tier import DeltaTier
import rospy
import threading
from minau.msg import ControlStatus
from etddf.msg import Measurement, MeasurementPackage, NetworkEstimate, AssetEstimate, EtddfStatistics
from etddf.srv import GetMeasurementPackage
import numpy as np
import tf
np.set_printoptions(suppress=True)
from copy import deepcopy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, Twist, Vector3, TwistWithCovariance
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from minau.msg import SonarTargetList, SonarTarget
from cuprint.cuprint import CUPrint

__author__ = "Luke Barbier"
__copyright__ = "Copyright 2020, COHRINT Lab"
__email__ = "luke.barbier@colorado.edu"
__status__ = "Development"
__license__ = "MIT"
__maintainer__ = "Luke Barbier"
__version__ = "1.2.0"

NUM_OWNSHIP_STATES = 6
DYNAMIC_VARIANCE = -1

# Design decision, do not instantiate asset (in the filter) until have first measurement of them

class ETDDF_Node:

    def __init__(self, my_name, \
                update_rate, \
                delta_tiers, \
                asset2id, \
                delta_codebook_table, \
                buffer_size, \
                meas_space_table, \
                missed_meas_tolerance_table, \
                x0,\
                P0,\
                Q,\
                default_meas_variance,
                use_control_input):

        self.update_rate = update_rate
        self.asset2id = asset2id
        self.Q = Q
        self.use_control_input = use_control_input
        self.default_meas_variance = default_meas_variance
        self.my_name = my_name
        
        self.filter = DeltaTier(NUM_OWNSHIP_STATES, \
                                x0,\
                                P0,\
                                buffer_size,\
                                meas_space_table,\
                                missed_meas_tolerance_table, \
                                delta_codebook_table,\
                                delta_tiers,\
                                self.asset2id,\
                                my_name)
                                                
        self.network_pub = rospy.Publisher("etddf/estimate/network", NetworkEstimate, queue_size=10)
        self.statistics_pub = rospy.Publisher("etddf/statistics", EtddfStatistics, queue_size=10)
        self.statistics = EtddfStatistics(0, rospy.get_rostime(), 0, 0, delta_tiers, [0 for _ in delta_tiers], 0.0, [], False)

        self.asset_pub_dict = {}
        for asset in self.asset2id.keys():
            if "surface" in asset:
                continue
            self.asset_pub_dict[asset] = rospy.Publisher("etddf/estimate/" + asset, Odometry, queue_size=10)        

        self.update_seq = 0
        self.last_depth_meas = None
        rospy.sleep(rospy.Duration(1 / self.update_rate))
        self.last_update_time = rospy.get_rostime() - rospy.Duration(1 / self.update_rate)
        self.meas_lock = threading.Lock()
        self.update_lock = threading.Lock()
        self.last_orientation = None

        # Initialize Measurement Callbacks
        # rospy.Subscriber("mavros/global_position/local", Odometry, self.depth_callback, queue_size=1)
        rospy.Subscriber("etddf/packages_in", MeasurementPackage, self.meas_pkg_callback, queue_size=1)

        if self.use_control_input:
            rospy.Subscriber("uuv_control/control_status", ControlStatus, self.control_status_callback, queue_size=1)

        # IMU Covariance Intersection
        if rospy.get_param("~measurement_topics/imu_ci") == "None":
            rospy.Timer(rospy.Duration(1 / self.update_rate), self.no_nav_filter_callback)
        else:
            rospy.Subscriber(rospy.get_param("~measurement_topics/imu_ci"), Odometry, self.nav_filter_callback, queue_size=1)

        # Sonar Subscription
        if rospy.get_param("~measurement_topics/sonar") != "None":
            rospy.Subscriber(rospy.get_param("~measurement_topics/sonar"), SonarTargetList, self.sonar_callback)

        # Initialize Buffer Service
        rospy.Service('etddf/get_measurement_package', GetMeasurementPackage, self.get_meas_pkg_callback)
        self.cuprint = CUPrint(rospy.get_name())
        self.cuprint("loaded")

    def sonar_callback(self, sonar_list):

        for target in sonar_list.targets:

            if self.last_orientation is None: # No orientation, no linearization of the sonar measurement
                return
            # Convert quaternions to Euler angles.
            (r, p, y) = tf.transformations.euler_from_quaternion([self.last_orientation.x, \
                self.last_orientation.y, self.last_orientation.z, self.last_orientation.w])
            bearing_world = y + target.bearing_rad

            z = target.range_m * np.sin(target.elevation_rad)
            xy_dist = target.range_m * np.cos(target.elevation_rad)
            x = xy_dist * np.cos(bearing_world)
            y = xy_dist * np.sin(bearing_world)

            now = rospy.get_rostime()
            sonar_x = Measurement("sonar_x", now, self.my_name, target.id, x, self.default_meas_variance["sonar_x"], [])
            sonar_y = Measurement("sonar_y", now, self.my_name, target.id, y, self.default_meas_variance["sonar_y"], [])
            sonar_z = Measurement("sonar_z", now, self.my_name, target.id, z, self.default_meas_variance["sonar_z"], [])

            self.filter.add_meas(sonar_x)
            self.filter.add_meas(sonar_y)
            self.filter.add_meas(sonar_z)

    def publish_stats(self, last_update_time):
        self.statistics.seq = self.update_seq
        self.statistics.stamp = last_update_time
        overflown, delta, buffer = self.filter.peek_buffer()
        self.statistics.current_lowest_multiplier = delta
        meas_name_list = [x.meas_type for x in buffer]
        self.statistics.current_lowest_buffer = meas_name_list
        self.overflown = overflown
        self.statistics_pub.publish(self.statistics)

    def no_nav_filter_callback(self, event):
        t_now = rospy.get_rostime()
        delta_t_ros =  t_now - self.last_update_time
        self.update_lock.acquire()

        ### Run Prediction ###
        if self.use_control_input:
            raise NotImplementedError("using control input not ready yet")
        else:
            self.filter.predict(np.zeros((3,1)), self.Q, delta_t_ros.to_sec(), False)

        ### Run Correction ###

        # Construct depth measurement
        z_r = self.default_meas_variance["depth"]
        z_data = self.last_depth_meas
        if z_data != None:
            z = Measurement("depth", t_now, self.my_name,"", z_data, z_r, [])
            self.filter.add_meas(z)
            self.last_depth_meas = None

        # correction
        self.filter.correct(t_now)
        self.publish_estimates(t_now, Odometry())
        self.last_update_time = t_now
        self.update_seq += 1
        self.update_lock.release()
        self.publish_stats(t_now)

    def nav_filter_callback(self, odom):

        # Update at specified rate
        t_now = rospy.get_rostime()
        delta_t_ros =  t_now - self.last_update_time
        if delta_t_ros < rospy.Duration(1/self.update_rate):
            return

        self.update_lock.acquire()

        ### Run Prediction ###
        if self.use_control_input:
            raise NotImplementedError("using control input not ready yet")
        else:
            self.filter.predict(np.zeros((3,1)), self.Q, delta_t_ros.to_sec(), False)

        ### Run Correction ###

        # Construct depth measurement
        z_r = self.default_meas_variance["depth"]
        z_data = self.last_depth_meas
        if z_data != None:
            z = Measurement("depth", t_now, self.my_name,"", z_data, z_r, [])
            self.filter.add_meas(z)
            self.last_depth_meas = None

        # correction
        self.filter.correct(t_now)

        ### Covariancee Intersect ###

        # Turn odom estimate into numpy
        mean = np.array([[ odom.pose.pose.position.x, \
                        odom.pose.pose.position.y, \
                        odom.pose.pose.position.z, \
                        odom.twist.twist.linear.x, \
                        odom.twist.twist.linear.y, \
                        odom.twist.twist.linear.z ]]).T
        cov_point = np.array(odom.pose.covariance).reshape(6,6)
        cov_twist = np.array(odom.twist.covariance).reshape(6,6)
        cov = np.zeros((NUM_OWNSHIP_STATES, NUM_OWNSHIP_STATES))
        cov[:3, :3] = cov_point[:3,:3]
        cov[3:, 3:] = cov_twist[3:,3:]

        # Run covariance intersection
        c_bar, Pcc = self.filter.intersect(mean, cov)

        # TODO partial state update everything

        self.last_orientation = odom.pose.pose.orientation
        self.publish_estimates(t_now, odom)
        self.last_update_time = t_now
        self.update_seq += 1
        self.update_lock.release()
        self.publish_stats(t_now)
    
    def control_status_callback(self, msg):
        self.meas_lock.acquire()
        # GRAB CONTROL INPUT
        self.meas_lock.release()

    def depth_callback(self, msg):
        self.meas_lock.acquire()
        self.last_depth_meas = msg.pose.pose.position.z
        self.meas_lock.release()

    def publish_estimates(self, timestamp, nav_estimate):
        ne = NetworkEstimate()
        for asset in self.asset2id.keys():
            if "surface" in asset:
                continue
            # else:
            #     print("publishing " + asset + "'s estimate")

            # Construct Odometry Msg for Asset
            nav_covpt = np.array(nav_estimate.pose.covariance).reshape(6,6)
            nav_covtw = np.array(nav_estimate.twist.covariance).reshape(6,6)

            mean, cov = self.filter.get_asset_estimate(asset)
            pose = Pose(Point(mean[0],mean[1],mean[2]), \
                        nav_estimate.pose.pose.orientation)
            pose_cov = np.zeros((6,6))
            pose_cov[:3,:3] = cov[:3,:3]
            pose_cov[3:,3:] = nav_covpt[3:,3:]
            pwc = PoseWithCovariance(pose, list(pose_cov.flatten()))

            tw = Twist(Vector3(mean[3],mean[4],mean[5]), nav_estimate.twist.twist.angular)
            twist_cov = np.zeros((6,6))
            twist_cov[:3,:3] = cov[3:6,3:6]
            twist_cov[3:,3:] = nav_covtw[3:,3:]
            twc = TwistWithCovariance(tw, list(twist_cov.flatten()))
            h = Header(self.update_seq, timestamp, "map")
            o = Odometry(h, "map", pwc, twc)

            ae = AssetEstimate(o, asset)
            ne.assets.append(ae)
            self.asset_pub_dict[asset].publish(o)

        self.network_pub.publish(ne)

    def meas_pkg_callback(self, msg):

        # Modem update
        if msg.src_asset == "surface" or msg.src_asset == self.my_name:
            for meas in msg.measurements:
                # Approximate the fuse on the next update, so we can get other asset's position immediately
                self.filter.add_meas(meas, force_fuse=True)
        else:
            self.update_lock.acquire()
            implicit_cnt, explicit_cnt = self.filter.catch_up(msg.delta_multiplier, msg.measurements)
            self.update_lock.release()
            self.statistics.implicit_count += implicit_cnt
            self.statistics.explicit_count += explicit_cnt

    def get_meas_pkg_callback(self, req):
        delta, buffer = self.filter.pull_buffer()
        ind = self.statistics.delta_tiers.index(delta)
        self.statistics.buffer_counts[ind] += 1
        return MeasurementPackage(buffer, self.my_name, delta)


################################
### Initialization Functions ###
################################

def get_indices_from_asset_names():
    my_name = rospy.get_param("~my_name")
    blue_team = rospy.get_param("~blue_team_names")
    red_team = rospy.get_param("~red_team_names")
    asset2id = {}
    asset2id[my_name] = 0

    next_index = 1
    for asset in blue_team:
        if asset == my_name:
            continue
        else:
            asset2id[asset] = next_index
            next_index += 1
    for asset in red_team:
        asset2id[asset] = next_index
        next_index += 1

    asset2id["surface"] = -1 # arbitrary negative number

    return asset2id

def get_delta_codebook_table():
    delta_codebook = {}

    meas_info = rospy.get_param("~measurements")
    for meas in meas_info.keys():
        base_et_delta = meas_info[meas]["base_et_delta"]
        delta_codebook[meas] = base_et_delta
    return delta_codebook

def get_missed_meas_tolerance_table():
    meas_tolerance_table = {}

    meas_info = rospy.get_param("~measurements")
    for meas in meas_info.keys():
        meas_tolerance_table[meas] = meas_info[meas]["missed_tolerance"]

    return meas_space_table

def get_meas_space_table():
    meas_space_table = {}

    meas_info = rospy.get_param("~measurements")
    for meas in meas_info.keys():
        meas_space_table[meas] = meas_info[meas]["buffer_size"]

    meas_space_table["bookstart"] = rospy.get_param("~buffer_space/bookstart")
    meas_space_table["bookend"] = rospy.get_param("~buffer_space/bookend")
    meas_space_table["final_time"] = rospy.get_param("~buffer_space/final_time")

    return meas_space_table

def _dict2arr(d):
    return np.array([[d["x"]],\
                    [d["y"]],\
                    [d["z"]],\
                    [d["x_vel"]], \
                    [d["y_vel"]],\
                    [d["z_vel"]]])
def _list2arr(l):
    return np.array([l]).reshape(-1,1)

def _add_velocity_states(base_states):
    velocities = np.zeros((base_states.size,1))
    return np.concatenate((base_states, velocities), axis=0)

def get_initial_estimate(num_states):
    default_starting_position = _dict2arr(rospy.get_param("~default_starting_position"))
    uncertainty_known_starting_position = _dict2arr( rospy.get_param("~initial_uncertainty/known_starting_position"))
    uncertainty_unknown_starting_position = _dict2arr( rospy.get_param("~initial_uncertainty/unknown_starting_position"))

    my_starting_position = rospy.get_param("~starting_position")
    if not my_starting_position:
        my_starting_position = deepcopy(default_starting_position)
    else:
        my_starting_position = _add_velocity_states( _list2arr(my_starting_position))
    ownship_uncertainty = _dict2arr( rospy.get_param("~initial_uncertainty/ownship") )

    uncertainty = np.zeros((num_states,num_states))
    uncertainty_vector = np.zeros((num_states,1))
    uncertainty_vector[:NUM_OWNSHIP_STATES] = ownship_uncertainty
    uncertainty += np.eye(num_states) * uncertainty_vector

    state_vector = my_starting_position
    blue_team_names = rospy.get_param("~blue_team_names")
    blue_team_positions = rospy.get_param("~blue_team_positions")
    red_team_names = rospy.get_param("~red_team_names")

    next_index = 1
    for asset in blue_team_names:
        if len(blue_team_positions) >= next_index: # we were given the positione of this asset in roslaunch
            next_position = _add_velocity_states( _list2arr( blue_team_positions[next_index-1]))
            uncertainty_vector = np.zeros((num_states,1))
            uncertainty_vector[next_index*NUM_OWNSHIP_STATES:(next_index+1)*NUM_OWNSHIP_STATES] = uncertainty_known_starting_position
            uncertainty += np.eye(num_states) * uncertainty_vector
        else:
            next_position = deepcopy(default_starting_position)
            uncertainty_vector = np.zeros((num_states,1))
            uncertainty_vector[next_index*NUM_OWNSHIP_STATES:(next_index+1)*NUM_OWNSHIP_STATES] = uncertainty_unknown_starting_position
            uncertainty += np.eye(num_states) * uncertainty_vector

        state_vector = np.concatenate((state_vector, next_position),axis=0)
        next_index += 1
    for asset in red_team_names:
        next_position = deepcopy(default_starting_position)
        state_vector = np.concatenate((state_vector, next_position),axis=0)

        uncertainty_vector = np.zeros((num_states,1))
        uncertainty_vector[next_index*NUM_OWNSHIP_STATES:(next_index+1)*NUM_OWNSHIP_STATES] = uncertainty_unknown_starting_position
        uncertainty += np.eye(num_states) * uncertainty_vector

        next_index += 1
    
    return state_vector, uncertainty

def get_process_noise(num_states):
    Q = np.zeros((num_states, num_states))
    ownship_Q = _dict2arr(rospy.get_param("~process_noise/ownship"))
    blueteam_Q = _dict2arr(rospy.get_param("~process_noise/blueteam"))
    redteam_Q = _dict2arr(rospy.get_param("~process_noise/redteam"))

    Q_vec = np.zeros((num_states,1))
    Q_vec[:NUM_OWNSHIP_STATES] = ownship_Q
    Q += np.eye(num_states) * Q_vec

    blue_team_names = rospy.get_param("~blue_team_names")
    red_team_names = rospy.get_param("~red_team_names")

    next_index = 1
    for asset in blue_team_names:
        Q_vec = np.zeros((num_states,1))
        Q_vec[next_index*NUM_OWNSHIP_STATES:(next_index+1)*NUM_OWNSHIP_STATES] = blueteam_Q
        Q += np.eye(num_states) * Q_vec
        next_index += 1
    for asset in red_team_names:
        Q_vec = np.zeros((num_states,1))
        Q_vec[next_index*NUM_OWNSHIP_STATES:(next_index+1)*NUM_OWNSHIP_STATES] = redteam_Q
        Q += np.eye(num_states) * Q_vec
        next_index += 1
    return Q

def get_default_meas_variance():
    meas_vars = {}
    meas_info = rospy.get_param("~measurements")
    for meas in meas_info.keys():
        sd = meas_info[meas]["default_sd"]
        meas_vars[meas] = sd ** 2 if type(sd) != str else DYNAMIC_VARIANCE
    return meas_vars

if __name__ == "__main__":
    rospy.init_node("etddf_node")
    my_name = rospy.get_param("~my_name")
    update_rate = rospy.get_param("~update_rate")
    delta_tiers = rospy.get_param("~delta_tiers")
    asset2id = get_indices_from_asset_names()
    delta_codebook_table = get_delta_codebook_table()
    buffer_size = rospy.get_param("~buffer_space/capacity")
    meas_space_table = get_meas_space_table()
    missed_meas_tolerance_table = get_missed_meas_tolerance_table()
    num_assets = len(asset2id) - 1 # subtract surface
    x0, P0 = get_initial_estimate(num_assets * NUM_OWNSHIP_STATES)
    Q = get_process_noise(num_assets * NUM_OWNSHIP_STATES)
    default_meas_variance = get_default_meas_variance()
    use_control_input = rospy.get_param("~use_control_input")

    et_node = ETDDF_Node(my_name,
                        update_rate, \
                        delta_tiers, \
                        asset2id, \
                        delta_codebook_table, \
                        buffer_size, \
                        meas_space_table, \
                        missed_meas_tolerance_table, \
                        x0,\
                        P0,\
                        Q,\
                        default_meas_variance,\
                        use_control_input)

    rospy.spin()