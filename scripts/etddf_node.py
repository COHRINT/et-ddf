#!/usr/bin/env python
from __future__ import division

from etddf.delta_tier_filter import MinauDeltaTierFilter
import rospy
import threading
from minau.msg import ControlStatus
from etddf.msg import Measurement, MeasurementPackage, NetworkEstimate, AssetEstimate
import numpy as np
from copy import deepcopy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, Twist, Vector3, TwistWithCovariance
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

NUM_OWNSHIP_STATES = 8
DYNAMIC_VARIANCE = -1

# Design decision, do not instantiate asset until have first measurement of them

class ETDDF_Node:

    def __init__(self, my_name, update_rate, num_delta_tiers, \
                names2indices, \
                delta_codebook_hash, \
                buffer_size, \
                meas_buffer_hash, \
                x0,\
                P0,\
                Q,\
                default_meas_variance):
        self.my_name = my_name
        self.update_rate = update_rate
        self.last_update_time = rospy.get_rostime()
        self.names2indices = names2indices
        self.num_ownship_states = NUM_OWNSHIP_STATES
        self.num_states = self.num_ownship_states * len(self.names2indices)
        self.buffer_size = buffer_size
        self.meas_buffer_hash = meas_buffer_hash
        self.delta_codebook_hash = delta_codebook_hash
        self.update_seq = 0
        self.update_ledger = []
        self.default_meas_variance = default_meas_variance
        self.process_noise = Q
        
        # make copies & initialize mtdtf's
        self.main_fitler = MinauDeltaTierFilter(self.num_ownship_states, \
                                                x0,\
                                                P0,\
                                                remaining_meas_buffer_size = self.buffer_size,\
                                                meas_buffer_hash = self.meas_buffer_hash,\
                                                delta_codebook_hash = self.delta_codebook_hash[0],\
                                                delta_number = 0,\
                                                is_main_filter=True)
        self.main_fitler_copy = deepcopy(self.main_fitler)
        self.network_pub = rospy.Publisher("etddf/estimate/network", NetworkEstimate, queue_size=10)
        self.asset_pub_dict = {}
        for asset in self.names2indices.keys():
            if "surface" in asset:
                continue
            self.asset_pub_dict[asset] = rospy.Publisher("etddf/estimate/" + asset, Odometry, queue_size=10)

        rospy.Subscriber("etddf/packages_in", MeasurementPackage, self.meas_pkg_callback)
        rospy.Subscriber("uuv_control/control_status", ControlStatus, self.control_status_callback, queue_size=1)
        # Initialize Measurement Callbacks
        self.last_depth_meas = None
        self.meas_lock = threading.Lock()
        self.update_lock = threading.Lock()
        rospy.loginfo("etddf node loaded")

    
    def control_status_callback(self, msg):
        # Update at specified rate
        delta_t_ros = msg.header.stamp - self.last_update_time
        if delta_t_ros < rospy.Duration(1/self.update_rate):
            return
        
        self.meas_lock.acquire(True)
        rospy.loginfo("updating...")
        # Grab setpoint_velocity, depth and yaw from msg
        if msg.is_heading_velocity_setpoint_active:
            raise NotImplementedError("u is in NED coords now! and x,y,z now! Get velocity/heading setpoint as well?")
            u = np.array([[msg.setpoint_velocity.x, msg.setpoint_velocity.y, msg.setpoint_velocity.z]]).T
        else:
            u = np.zeros((3,1))
        depth = msg.current_position.z
        yaw = msg.current_heading * (np.pi / 180)
        self.meas_lock.release()
        depth_ros = Measurement("depth", \
                                msg.header.stamp, \
                                "",\
                                "",\
                                depth, \
                                self.default_meas_variance["depth"], 
                                [])
        yaw_ros = Measurement("yaw", \
                                msg.header.stamp, \
                                "",\
                                "",\
                                yaw, \
                                self.default_meas_variance["yaw"], 
                                [])
        self.main_fitler.receive_meas(depth_ros, 0, 0)
        self.main_fitler.receive_meas(yaw_ros, 0, 0)
        self.update_lock.acquire(True)
        self.main_fitler.predict(u, self.process_noise)
        self.main_fitler.correct()
        self.update_ledger.append([msg.header.stamp, self.update_seq])

        # grab update & meas lock
        # add all measurements to mdtfs

        # delete those that overflow
        ### TODO add logic for no remaining consistent buffers
        # release meas lock
        # predict all mtdf's
        # correct all mtdf's
        # publish main estimate
        self.last_update_time = msg.header.stamp
        self.publish_estimates(msg.header.stamp)
        self.update_seq += 1
        self.update_lock.release()

    # def depth_callback(self, msg): # TODO grab depth from pixhawk
    #     self.meas_lock.acquire(blocking=True)
    #     self.last_depth_meas = msg
    #     self.meas_lock.release()

    def publish_estimates(self, timestamp):
        ne = NetworkEstimate()
        for asset in self.names2indices.keys():
            if "surface" in asset:
                continue
            ind = self.names2indices[asset]

            # Construct Odometry Msg for Asset
            mean, cov = self.main_fitler.get_asset_estimate(ind)
            yaw = mean[3]
            quat = quaternion_from_euler(0, 0, yaw)
            pose = Pose(Point(mean[0],mean[1],mean[2]), \
                        Quaternion(quat[0],quat[1],quat[2],quat[3]))
            pose_cov = np.zeros((6,6))
            pose_cov[:3,:3] = cov[:3,:3]
            pose_cov[5,:3] = cov[3,:3]
            pose_cov[:3,5] = cov[:3,3]
            pose_cov[5,5] = cov[3,3]
            pwc = PoseWithCovariance(pose, list(pose_cov.flatten()))
            tw = Twist(Vector3(mean[4],mean[5],mean[6]), Vector3(0,0,mean[7]))
            twist_cov = np.zeros((6,6))
            twist_cov[:3,:3] = cov[4:7,4:7]
            twist_cov[:3,5] = cov[:3,7]
            twist_cov[5,:3] = cov[7,:3]
            twist_cov[5,5] = cov[7,7]
            twc = TwistWithCovariance(tw, list(twist_cov.flatten()))
            h = Header(self.update_seq, timestamp, "world_ned")
            o = Odometry(h, asset+"/base_link", pwc, twc)

            ae = AssetEstimate(o, asset)
            ne.assets.append(ae)
            self.asset_pub_dict[asset].publish(o)

        self.network_pub.publish(ne)

    def meas_pkg_callback(self, msg):
        # determine if it's shared OR if it's our own modem that took the measurement
        pass

    def get_meas_pkg_callback(sel, msg):
        pass
    
def get_indices_from_asset_names():
    my_name = rospy.get_param("~my_name")
    blue_team = rospy.get_param("~blue_team_names")
    red_team = rospy.get_param("~red_team_names")
    names2indices = {}
    names2indices[my_name] = 0

    next_index = 1
    for asset in blue_team:
        if asset == my_name:
            continue
        else:
            names2indices[asset] = next_index
            next_index += 1
    for asset in red_team:
        names2indices[asset] = next_index
        next_index += 1

    names2indices["surface"] = -1 # arbitrary negative number

    return names2indices

def get_delta_codebook_hash(num_delta_tiers):
    delta_codebooks = [ {} for i in range(num_delta_tiers) ]

    meas_info = rospy.get_param("~measurements")
    for meas in meas_info.keys():
        low_delta = meas_info[meas]["low_et_delta"]
        high_delta = meas_info[meas]["high_et_delta"]
        deltas = np.linspace(low_delta, high_delta, num_delta_tiers)

        for i in range(num_delta_tiers):
            delta_codebooks[i][meas] = deltas[i]

    return delta_codebooks

def get_meas_buffer_hash():
    meas_buffer_hash = {}

    meas_info = rospy.get_param("~measurements")
    for meas in meas_info.keys():
        meas_buffer_hash[meas] = meas_info[meas]["buffer_size"]

    return meas_buffer_hash

def _dict2arr(d):
    return np.array([[d["x"]],\
                    [d["y"]],\
                    [d["z"]],\
                    [d["yaw"]],\
                    [d["body_forward_vel"]], \
                    [d["body_strafe_vel"]],\
                    [d["body_depth_vel"]],\
                    [d["body_yaw_vel"]]])
def _list2arr(l):
    return np.array([l]).reshape(-1,1)

def _add_velocity_states(base_states):
    velocities = np.zeros((base_states.size,1))
    return np.concatenate((base_states, velocities), axis=0)

def get_initial_estimate(num_states):
    default_starting_pose = _dict2arr(rospy.get_param("~default_starting_pose"))
    uncertainty_known_starting_pose = _dict2arr( rospy.get_param("~initial_uncertainty/known_starting_pose"))
    uncertainty_unknown_starting_pose = _dict2arr( rospy.get_param("~initial_uncertainty/unknown_starting_pose"))

    my_starting_pose = rospy.get_param("~starting_pose")
    if not my_starting_pose:
        my_starting_pose = deepcopy(default_starting_pose)
    else:
        my_starting_pose = _add_velocity_states( _list2arr(my_starting_pose))
    ownship_uncertainty = _dict2arr( rospy.get_param("~initial_uncertainty/ownship") )

    uncertainty = np.zeros((num_states,num_states))
    uncertainty_vector = np.zeros((num_states,1))
    uncertainty_vector[:NUM_OWNSHIP_STATES] = ownship_uncertainty
    uncertainty += np.eye(num_states) * uncertainty_vector

    state_vector = my_starting_pose
    blue_team_names = rospy.get_param("~blue_team_names")
    blue_team_poses = rospy.get_param("~blue_team_poses")
    red_team_names = rospy.get_param("~red_team_names")

    next_index = 0
    for asset in blue_team_names:
        if len(blue_team_poses) > next_index: # we were given the pose of this asset in roslaunch
            next_pose = _add_velocity_states( _list2arr( blue_team_poses[next_index]))
            uncertainty_vector = np.zeros((num_states,1))
            uncertainty_vector[next_index*NUM_OWNSHIP_STATES:(next_index+1)*NUM_OWNSHIP_STATES] = uncertainty_known_starting_pose
            uncertainty += np.eye(num_states) * uncertainty_vector
        else:
            next_pose = deepcopy(default_starting_pose)
            uncertainty_vector = np.zeros((num_states,1))
            uncertainty_vector[next_index*NUM_OWNSHIP_STATES:(next_index+1)*NUM_OWNSHIP_STATES] = uncertainty_unknown_starting_pose
            uncertainty += np.eye(num_states) * uncertainty_vector

        state_vector = np.concatenate((state_vector, next_pose),axis=0)
        next_index += 1
    for asset in red_team_names:
        next_pose = deepcopy(default_starting_pose)
        state_vector = np.concatenate((state_vector, next_pose),axis=0)

        uncertainty_vector = np.zeros((num_states,1))
        uncertainty_vector[next_index*NUM_OWNSHIP_STATES:(next_index+1)*NUM_OWNSHIP_STATES] = uncertainty_unknown_starting_pose
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

    next_index = 0 
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
    num_delta_tiers = rospy.get_param("~num_delta_tiers")    
    names2indices = get_indices_from_asset_names()
    delta_codebook_hash = get_delta_codebook_hash(num_delta_tiers)
    buffer_size = rospy.get_param("~buffer_size")
    meas_buffer_hash = get_meas_buffer_hash()
    num_assets = len(names2indices) - 1 # subtract surface
    x0, P0 = get_initial_estimate(num_assets * NUM_OWNSHIP_STATES)
    Q = get_process_noise(num_assets * NUM_OWNSHIP_STATES)
    default_meas_variance = get_default_meas_variance()

    print(Q)
    print(x0)
    print(P0)

    et_node = ETDDF_Node(my_name,
                        update_rate, \
                        num_delta_tiers, \
                        names2indices, \
                        delta_codebook_hash, \
                        buffer_size, \
                        meas_buffer_hash, \
                        x0,\
                        P0,\
                        Q,\
                        default_meas_variance)
    # print(update_rate)
    # load rosparams, initialize etddf_node
    # 

    rospy.spin()