from __future__ import division
from copy import deepcopy
import numpy as np

# Normalize Angle -pi to pi
def normalize_angle(angle):
    return np.mod( angle + np.pi, 2*np.pi) - np.pi
# Normalize all angles in our state
def normalize_all_angles(state_vector, num_ownship_states, num_assets, world_dim):
    state_vector = deepcopy(state_vector)
    if world_dim == 2 and num_ownship_states in [3,6]:
        for i in range(num_assets):
            asset_yaw_index = i*num_ownship_states + 2
            state_vector[asset_yaw_index,0] = normalize_angle(state_vector[asset_yaw_index,0])
    elif world_dim == 3 and num_ownship_states > 6:
        for i in range(num_assets):
            asset_yaw_index = i*num_ownship_states + 3
            state_vector[asset_yaw_index,0] = normalize_angle(state_vector[asset_yaw_index,0])
    return state_vector