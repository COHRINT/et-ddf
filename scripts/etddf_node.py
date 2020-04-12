#!/usr/bin/env python
from __future__ import division

from etddf.delta_tier_filter import MinauDeltaTierFilter
import rospy
import threading
from minau.msg import ControlStatus
from etddf.msg import Measurement, MeasurementPackage
import numpy as np
from copy import deepcopy

NUM_OWNSHIP_STATES = 8

# Design decision, do not instantiate asset until have first measurement of them

class ETDDF_Node:

    def __init__(self, update_rate, num_delta_tiers, \
                names2indices, \
                delta_codebook_hash, \
                meas_buffer_hash, \
                starting_position):
        self.update_rate = update_rate
        self.last_update_time = rospy.get_rostime()
        self.names2indices = names2indices
        self.num_ownship_states = NUM_OWNSHIP_STATES
        self.num_states = self.num_ownship_states * len(self.names2indices)

        
        # make copies & initialize mtdtf's
        self.main_fitler = MinauDeltaTierFilter(0, self.num_ownship_states, 3, )

        rospy.Subscriber("etddf/meas_pkgs_in", MeasurementPackage, self.meas_pkg_callback)
        rospy.Subscriber("uuv_control/control_status", ControlStatus, self.control_status_callback, queue_size=1)
        # Initialize Measurement Callbacks
        self.last_depth_meas = None
        self.meas_lock = threading.Lock()

        


    def control_status_callback(self, msg):
        # Update at specified rate
        delta_t_ros = msg.header.stamp - self.last_update_time
        if delta_t_ros < rospy.Duration(1/self.update_rate):
            return
        
        self.meas_lock.acquire(True)
        rospy.loginfo("updating...")
        

        # Grab setpoint_velocity, depth and yaw from msg
        if msg.is_heading_velocity_setpoint_active:
            u = np.array([[msg.setpoint_velocity.x, msg.setpoint_velocity.y, msg.setpoint_velocity.z]]).T
        else:
            u = np.zeros((3,1))
        depth = msg.current_position.z
        yaw = msg.current_heading * (np.pi / 180)
        
        self.meas_lock.release()

        # grab update & meas lock
        # add all measurements to mdtfs
        # delete those that overflow
        ### TODO add logic for no remaining consistent buffers
        # release meas lock
        # predict all mtdf's
        # correct all mtdf's
        self.last_update_time = msg.header.stamp

    # def depth_callback(self, msg): # TODO grab depth from pixhawk
    #     self.meas_lock.acquire(blocking=True)
    #     self.last_depth_meas = msg
    #     self.meas_lock.release()

    def meas_pkg_callback(self, msg):
        # determine if it's shared OR if it's our own modem that took the measurement
        pass

    def get_meas_pkg_callback(sel, msg):
        pass
    
def get_indices_from_asset_names(my_name, blue_team, red_team):
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

if __name__ == "__main__":
    rospy.init_node("etddf_node")
    update_rate = rospy.get_param("~update_rate")
    num_delta_tiers = rospy.get_param("~num_delta_tiers")
    my_name = rospy.get_param("~my_name")
    blue_team = rospy.get_param("~blue_team_names")
    red_team = rospy.get_param("~red_team_names")
    names2indices = get_indices_from_asset_names(my_name, blue_team, red_team)
    num_assets = len(names2indices)
    delta_codebook_hash = get_delta_codebook_hash(num_delta_tiers)
    meas_buffer_hash = get_meas_buffer_hash()
    starting_pose = rospy.get_param("~starting_pose")
    buffer_size = rospy.get_param("~buffer_size")
    x0, P0 = get_initial_estimate(num_assets * NUM_OWNSHIP_STATES)
    Q = get_process_noise(num_assets * NUM_OWNSHIP_STATES)

    # et_node = ETDDF_Node(update_rate, \
    #                     num_delta_tiers, \
    #                     names2indices, \
    #                     delta_codebook_hash, \
    #                     meas_buffer_hash, \
    #                     starting_pose,\
    #                     )
    # print(update_rate)
    # load rosparams, initialize etddf_node
    # 

    rospy.spin()