#!/usr/bin/env python
from __future__ import division
"""@package etddf

ROS interface script for delta tiering filter

Filter operates in ENU

"""

from etddf.delta_tier import DeltaTier
import rospy
import threading
from minau.msg import ControlStatus
from etddf.msg import Measurement, MeasurementPackage, NetworkEstimate, AssetEstimate, EtddfStatistics, PositionVelocity
from etddf.srv import GetMeasurementPackage
import numpy as np
import tf
np.set_printoptions(suppress=True)
from copy import deepcopy
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, Twist, Vector3, TwistWithCovariance, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from minau.msg import SonarTargetList, SonarTarget
from cuprint.cuprint import CUPrint

__author__ = "Luke Barbier"
__copyright__ = "Copyright 2020, COHRINT Lab"
__email__ = "luke.barbier@colorado.edu"
__status__ = "Development"
__license__ = "MIT"
__maintainer__ = "Luke Barbier"
__version__ = "3.0"

NUM_OWNSHIP_STATES = 6

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
        self.landmark_dict = rospy.get_param("~landmarks", {})

        self.cuprint = CUPrint(rospy.get_name())
        
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
        self.red_asset_found = False
        self.red_asset_names = rospy.get_param("~red_team_names")

        # Depth Sensor
        if rospy.get_param("~measurement_topics/depth") != "None":
            rospy.Subscriber(rospy.get_param("~measurement_topics/depth"), Float64, self.depth_callback, queue_size=1)

        # Modem & Measurement Packages
        rospy.Subscriber("etddf/packages_in", MeasurementPackage, self.meas_pkg_callback, queue_size=1)

        if self.use_control_input:
            self.control_input = None
            rospy.Subscriber("uuv_control/control_status", ControlStatus, self.control_status_callback, queue_size=1)

        
        if rospy.get_param("~strapdown"):
            rospy.Subscriber(rospy.get_param("~measurement_topics/imu_est"), Odometry, self.orientation_estimate_callback, queue_size=1)
            rospy.wait_for_message(rospy.get_param("~measurement_topics/imu_est"), Odometry)

        # IMU Covariance Intersection
        if rospy.get_param("~strapdown") and rospy.get_param("~measurement_topics/imu_ci") != "None":
            self.cuprint("Intersecting with strapdown")
            self.intersection_pub = rospy.Publisher("strapdown/intersection_result", PositionVelocity, queue_size=1)
            rospy.Subscriber(rospy.get_param("~measurement_topics/imu_ci"), PositionVelocity, self.nav_filter_callback, queue_size=1)
        else:
            self.cuprint("Not intersecting with strapdown filter")
            rospy.Timer(rospy.Duration(1 / self.update_rate), self.no_nav_filter_callback)

        # Sonar Subscription
        if rospy.get_param("~measurement_topics/sonar") != "None":
            rospy.Subscriber(rospy.get_param("~measurement_topics/sonar"), SonarTargetList, self.sonar_callback)
        
        self.data_x, self.data_y = None, None
        # rospy.Subscriber("pose_gt", Odometry, self.gps_callback, queue_size=1)
    
        # Initialize Buffer Service
        rospy.Service('etddf/get_measurement_package', GetMeasurementPackage, self.get_meas_pkg_callback)
        self.cuprint("loaded")

    def gps_callback(self, msg):
        self.data_x = msg.pose.pose.position.x + np.random.normal(0, scale=0.05)
        self.data_y = msg.pose.pose.position.y + np.random.normal(0, scale=0.05)

    def orientation_estimate_callback(self, odom):
        self.meas_lock.acquire()
        self.last_orientation = odom.pose.pose.orientation
        self.last_orientation_cov = np.array(odom.pose.covariance).reshape(6,6)
        self.last_orientation_dot = odom.twist.twist.angular
        self.last_orientation_dot_cov = np.array(odom.twist.covariance).reshape(6,6)
        self.meas_lock.release()

    def sonar_callback(self, sonar_list):

        for target in sonar_list.targets:
            # self.cuprint("Receiving sonar measurements")
            if self.last_orientation is None: # No orientation, no linearization of the sonar measurement
                # print("no ori")
                return
            if target.id == "detection":
                continue

            # self.cuprint("Receiving sonar data")
            # Convert quaternions to Euler angles.
            self.meas_lock.acquire()
            (r, p, y) = tf.transformations.euler_from_quaternion([self.last_orientation.x, \
                self.last_orientation.y, self.last_orientation.z, self.last_orientation.w])
            self.meas_lock.release()
            # y = (np.pi/180.0) * 8
            bearing_world = y + target.bearing_rad

            z = target.range_m * np.sin(target.elevation_rad)
            xy_dist = target.range_m * np.cos(target.elevation_rad)
            x = xy_dist * np.cos(bearing_world)
            y = xy_dist * np.sin(bearing_world)

            now = rospy.get_rostime()
            sonar_x, sonar_y = None, None
            if "landmark_" in target.id:
                sonar_x = Measurement("sonar_x", now, self.my_name, "", x, self.default_meas_variance["sonar_x"], self.landmark_dict[target.id[len("landmark_"):]])
                sonar_y = Measurement("sonar_y", now, self.my_name, "", y, self.default_meas_variance["sonar_x"], self.landmark_dict[target.id[len("landmark_"):]])
            else:
                sonar_x = Measurement("sonar_x", now, self.my_name, target.id, x, self.default_meas_variance["sonar_x"], [])
                sonar_y = Measurement("sonar_y", now, self.my_name, target.id, y, self.default_meas_variance["sonar_y"], [])
                if target.id in self.red_asset_names and not self.red_asset_found:
                    self.cuprint("Red Asset detected!")
                    self.red_asset_found = True
            # sonar_z = Measurement("sonar_z", now, self.my_name, target.id, z, self.default_meas_variance["sonar_z"], []

            self.filter.add_meas(sonar_x)
            self.filter.add_meas(sonar_y)
            # self.filter.add_meas(sonar_z)
            # self.cuprint("meas added")

    def publish_stats(self, last_update_time):
        self.statistics.seq = self.update_seq
        self.statistics.stamp = last_update_time
        self.statistics.overflown, delta, buffer = self.filter.peek_buffer()
        self.statistics.current_lowest_multiplier = delta
        meas_name_list = [x.meas_type for x in buffer]
        self.statistics.current_lowest_buffer = meas_name_list
        self.statistics_pub.publish(self.statistics)

    def no_nav_filter_callback(self, event):
        t_now = rospy.get_rostime()
        delta_t_ros =  t_now - self.last_update_time
        self.update_lock.acquire()

        ### Run Prediction ###
        ### Run Prediction ###
        if self.use_control_input and self.control_input is not None:
            self.filter.predict(self.control_input, self.Q, delta_t_ros.to_sec(), False)
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
        self.publish_estimates(t_now)
        self.last_update_time = t_now
        self.update_seq += 1
        self.update_lock.release()
        self.publish_stats(t_now)

    def nav_filter_callback(self, pv_msg):
        
        # Update at specified rate
        t_now = rospy.get_rostime()
        delta_t_ros =  t_now - self.last_update_time
        if delta_t_ros < rospy.Duration(1/self.update_rate):
            return

        self.update_lock.acquire()

        ### Run Prediction ###
        if self.use_control_input and self.control_input is not None:
            self.filter.predict(self.control_input, self.Q, delta_t_ros.to_sec(), False)
        else:
            self.filter.predict(np.zeros((3,1)), self.Q, delta_t_ros.to_sec(), False)

        ### Run Correction ###

        # Construct depth measurement
        z_r = self.default_meas_variance["depth"]
        z_data = self.last_depth_meas
        if z_data != None:
            z = Measurement("depth", t_now, self.my_name,"", z_data, z_r, []) # Flip z data to transform enu -> NED
            self.filter.add_meas(z)
            self.last_depth_meas = None
        if self.data_x != None:
            x = Measurement("gps_x", t_now, self.my_name,"", self.data_x, 0.1, [])
            self.filter.add_meas(x)
            self.data_x = None
        if self.data_y != None:
            y = Measurement("gps_y", t_now, self.my_name,"", self.data_y, 0.1, [])
            self.filter.add_meas(y)
            self.data_y = None

        # correction
        self.filter.correct(t_now)

        ### Covariancee Intersect ###

        # Turn odom estimate into numpy
        mean = np.array([[pv_msg.position.x, pv_msg.position.y, pv_msg.position.z, \
                        pv_msg.velocity.x, pv_msg.velocity.y, pv_msg.velocity.z]]).T
        cov = np.array(pv_msg.covariance).reshape(6,6)

        # Run covariance intersection
        c_bar, Pcc = self.filter.intersect(mean, cov)

        position = Vector3(c_bar[0,0], c_bar[1,0], c_bar[2,0])
        velocity = Vector3(c_bar[3,0], c_bar[4,0], c_bar[5,0])
        covariance = list(Pcc.flatten())
        new_pv_msg = PositionVelocity(position, velocity, covariance)
        self.intersection_pub.publish(new_pv_msg)

        self.publish_estimates(t_now)
        self.last_update_time = t_now
        self.update_seq += 1
        self.update_lock.release()
        self.publish_stats(t_now)
    
    def control_status_callback(self, msg):
        self.update_lock.acquire()
        if msg.is_setpoint_active and msg.is_heading_velocity_setpoint_active:
            self.control_input = np.array([[msg.setpoint_velocity.y, msg.setpoint_velocity.z, -msg.setpoint_velocity.z]]).T
        else:
            self.control_input = None
        # GRAB CONTROL INPUT
        self.update_lock.release()

    def depth_callback(self, msg):
        self.meas_lock.acquire()
        self.last_depth_meas = msg.data
        self.meas_lock.release()

    def publish_estimates(self, timestamp):
        ne = NetworkEstimate()
        for asset in self.asset2id.keys():
            if "surface" in asset:
                continue
            if "red" in asset and not self.red_asset_found:
                continue
            # else:
            #     print("publishing " + asset + "'s estimate")

            # Construct Odometry Msg for Asset

            mean, cov = self.filter.get_asset_estimate(asset)
            pose_cov = np.zeros((6,6))
            pose_cov[:3,:3] = cov[:3,:3]
            if asset == self.my_name:
                pose = Pose(Point(mean[0],mean[1],mean[2]), \
                            self.last_orientation)
                pose_cov[3:,3:] = self.last_orientation_cov[3:,3:]
            else:
                pose = Pose(Point(mean[0],mean[1],mean[2]), \
                            Quaternion(0,0,0,1))
                pose_cov[3:,3:] = np.eye(3) * 3
            pwc = PoseWithCovariance(pose, list(pose_cov.flatten()))

            twist_cov = np.zeros((6,6))
            twist_cov[:3,:3] = cov[3:6,3:6]
            if asset == self.my_name:
                tw = Twist(Vector3(mean[3],mean[4],mean[5]), self.last_orientation_dot)
                twist_cov[3:, 3:] = self.last_orientation_dot_cov[3:,3:]
            else:
                tw = Twist(Vector3(mean[3],mean[4],mean[5]), Vector3(0,0,0))
                twist_cov[3:, 3:] = np.eye(3) * -1
            twc = TwistWithCovariance(tw, list(twist_cov.flatten()))
            h = Header(self.update_seq, timestamp, "map")
            o = Odometry(h, "map", pwc, twc)

            ae = AssetEstimate(o, asset)
            ne.assets.append(ae)
            self.asset_pub_dict[asset].publish(o)

        self.network_pub.publish(ne)

    def meas_pkg_callback(self, msg):
        # Modem Meas taken by surface
        if msg.src_asset == "surface":
            self.cuprint("Receiving Surface Modem Measurements")
            for meas in msg.measurements:
                # Approximate the fuse on the next update, so we can get other asset's position immediately
                if meas.meas_type == "modem_elevation":
                    rospy.logerr("Ignoring Modem Elevation Measurement since we have depth measurements")
                    continue
                elif meas.meas_type == "modem_azimuth":
                    meas.global_pose = list(meas.global_pose)
                    # self.cuprint("azimuth: " + str(meas.data))
                    meas.data = (meas.data * np.pi) / 180
                    meas.variance = self.default_meas_variance["modem_azimuth"]
                elif meas.meas_type == "modem_range":
                    meas.global_pose = list(meas.global_pose)
                    # self.cuprint("range: " + str(meas.data))
                    meas.variance = self.default_meas_variance["modem_range"]
                self.filter.add_meas(meas, force_fuse=True)

        # Modem Meas taken by me
        elif msg.src_asset == self.my_name:
            # self.cuprint("Receiving Modem Measurements Taken by Me")
            for meas in msg.measurements:
                # Approximate the fuse on the next update, so we can get other asset's position immediately
                if meas.meas_type == "modem_elevation":
                    rospy.logerr("Ignoring Modem Elevation Measurement since we have depth measurements")
                    continue
                elif meas.meas_type == "modem_azimuth":
                    meas.global_pose = list(meas.global_pose)
                    meas.data = (meas.data * np.pi) / 180
                    meas.variance = self.default_meas_variance["modem_azimuth"]
                elif meas.meas_type == "modem_range":
                    meas.global_pose = list(meas.global_pose)
                    meas.variance = self.default_meas_variance["modem_range"]
                self.filter.add_meas(meas, force_fuse=True)

        # Buffer
        else:
            self.cuprint("receiving buffer")
            self.update_lock.acquire()

            # Loop through buffer and see if we've found the red agent
            for m in msg.measurements:
                if m.measured_asset in self.red_asset_names and not self.red_asset_found:
                    self.red_asset_found = True
                    self.cuprint("Red asset measurement received!")
            implicit_cnt, explicit_cnt = self.filter.catch_up(msg.delta_multiplier, msg.measurements)
            self.cuprint("...caught up")
            self.update_lock.release()
            self.statistics.implicit_count += implicit_cnt
            self.statistics.explicit_count += explicit_cnt

    def get_meas_pkg_callback(self, req):
        self.cuprint("pulling buffer")
        delta, buffer = self.filter.pull_buffer()
        ind = self.statistics.delta_tiers.index(delta)
        self.statistics.buffer_counts[ind] += 1
        mp = MeasurementPackage(buffer, self.my_name, delta)
        print(mp)
        return mp


################################
### Initialization Functions ###
################################

def get_indices_from_asset_names(blue_team):
    my_name = rospy.get_param("~my_name")
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

    if my_name != "surface":
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

    return meas_tolerance_table

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

def get_initial_estimate(num_states, blue_team_names, blue_team_positions):
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
    my_name = rospy.get_param("~my_name")
    red_team_names = rospy.get_param("~red_team_names")

    next_index_unc = 1
    next_index_pos = 1
    for asset in blue_team_names:
        if asset == my_name:
            next_index_pos += 1
            continue
        if len(blue_team_positions) >= next_index_pos: # we were given the positione of this asset in roslaunch
            next_position = _add_velocity_states( _list2arr( blue_team_positions[next_index_pos-1]))
            uncertainty_vector = np.zeros((num_states,1))
            uncertainty_vector[next_index_unc*NUM_OWNSHIP_STATES:(next_index_unc+1)*NUM_OWNSHIP_STATES] = uncertainty_known_starting_position
            uncertainty += np.eye(num_states) * uncertainty_vector
        else:
            next_position = deepcopy(default_starting_position)
            uncertainty_vector = np.zeros((num_states,1))
            uncertainty_vector[next_index_unc*NUM_OWNSHIP_STATES:(next_index_unc+1)*NUM_OWNSHIP_STATES] = uncertainty_unknown_starting_position
            uncertainty += np.eye(num_states) * uncertainty_vector

        state_vector = np.concatenate((state_vector, next_position),axis=0)
        next_index_unc += 1
        next_index_pos += 1
    for asset in red_team_names:
        next_position = deepcopy(default_starting_position)
        state_vector = np.concatenate((state_vector, next_position),axis=0)

        uncertainty_vector = np.zeros((num_states,1))
        uncertainty_vector[next_index_unc*NUM_OWNSHIP_STATES:(next_index_unc+1)*NUM_OWNSHIP_STATES] = uncertainty_unknown_starting_position
        uncertainty += np.eye(num_states) * uncertainty_vector

        next_index_unc += 1
    
    return state_vector, uncertainty

def get_process_noise(num_states, blue_team_names):
    Q = np.zeros((num_states, num_states))
    ownship_Q = _dict2arr(rospy.get_param("~process_noise/ownship"))
    blueteam_Q = _dict2arr(rospy.get_param("~process_noise/blueteam"))
    redteam_Q = _dict2arr(rospy.get_param("~process_noise/redteam"))

    Q_vec = np.zeros((num_states,1))
    Q_vec[:NUM_OWNSHIP_STATES] = ownship_Q
    Q += np.eye(num_states) * Q_vec

    my_name = rospy.get_param("~my_name")
    red_team_names = rospy.get_param("~red_team_names")

    next_index = 1
    for asset in blue_team_names:
        if asset == my_name:
            continue
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
        meas_vars[meas] = sd ** 2
    return meas_vars

if __name__ == "__main__":
    rospy.init_node("etddf_node")
    my_name = rospy.get_param("~my_name")
    update_rate = rospy.get_param("~update_rate")
    delta_tiers = rospy.get_param("~delta_tiers")
    blue_team_names = rospy.get_param("~blue_team_names")
    blue_team_positions = rospy.get_param("~blue_team_positions")

    # Don't track surface if it isn't this agent
    if my_name != "surface":
        ind = blue_team_names.index("surface")
        if ind >= 0:
            blue_team_names.pop(ind)
            blue_team_positions.pop(ind)


    asset2id = get_indices_from_asset_names(blue_team_names)
    delta_codebook_table = get_delta_codebook_table()
    buffer_size = rospy.get_param("~buffer_space/capacity")
    meas_space_table = get_meas_space_table()
    missed_meas_tolerance_table = get_missed_meas_tolerance_table()
    if my_name != "surface":
        num_assets = len(asset2id) - 1 # subtract surface
    else:
        num_assets = len(asset2id)
    x0, P0 = get_initial_estimate(num_assets * NUM_OWNSHIP_STATES, blue_team_names, blue_team_positions)
    Q = get_process_noise(num_assets * NUM_OWNSHIP_STATES, blue_team_names)
    rospy.logwarn("{}, {}, {}, {}".format(my_name, x0.shape, P0.shape, Q.shape))
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