from __future__ import division
"""@package etddf

State propagation functions

"""
__author__ = "Luke Barbier"
__copyright__ = "Copyright 2020, COHRINT Lab"
__email__ = "luke.barbier@colorado.edu"
__status__ = "Stable"
__maintainer__ = "Luke Barbier"

import numpy as np
from copy import deepcopy

def linear_propagation(x_hat, u, num_ownship_states, my_id, time_delta=1.0, use_control_input=False):
    """Propagates the state

    Control input can optionally be used for my_id
    Assumes constant velocity model for all other assets

    Arguments:
        x_hat {np.array} -- State vector (num_states,1)
        u {np.array} -- Control Input (3x1; x,y,z)
        num_ownship_states {int} -- number of ownship states
        my_id {innt} -- ID of asset for which control input corresponds

    Keyword Arguments:
        time_delta {float} -- amount of time to propagate the step (default: {1.0})
        use_control_input {bool} -- whether to use control input or assume constant velocity (default: {False})

    Returns:
        np.array -- propagated state
        np.array -- propagated uncertainty
    """
    num_states = x_hat.size
    num_assets = int(num_states / num_ownship_states)
    x_hat = deepcopy(x_hat)

    A = np.eye(num_states)
    num_base_states = int(num_ownship_states / 2)
    for i in range(num_assets):
        for d in range(num_base_states):
            A[i*num_ownship_states+d, i*num_ownship_states + num_base_states + d] = time_delta

    x_new = None
    if use_control_input:
        B = np.zeros((num_states,1))
        B[my_id*num_ownship_states,0] = u[0,0] * time_delta
        B[my_id*num_ownship_states+1,0] = u[1,0] * time_delta
        B[my_id*num_ownship_states+2,0] = u[2,0] * time_delta
        x_new = A.dot(x_hat) + B
        x_new[my_id*num_ownship_states+num_base_states:(my_id+1)*num_ownship_states] = u[:]
        A[my_id*num_ownship_states,my_id*num_ownship_states + num_base_states] = 1
        A[my_id*num_ownship_states+1,my_id*num_ownship_states+1 + num_base_states] = 1
        A[my_id*num_ownship_states+2,my_id*num_ownship_states+2 + num_base_states] = 1
    else:
        x_new = A.dot(x_hat)
    return x_new, A

# def nonlinear_propagation(self, u):
#     speed_index, angular_vel_index = None, None
#     u_speed_index, u_ang_vel_index = None, None
#     theta_index = None

#     # Configure Indices of velocities so code remains valid for 2D and 3D
#     if world_dim == 2:
#         speed_index = 3
#         angular_vel_index = 5
#         u_speed_index = 0
#         u_ang_vel_index = 1
#         theta_index = 2
#     else: # world dim 3
#         speed_index = 4
#         angular_vel_index = 7
#         u_speed_index = 0
#         u_ang_vel_index = 2
#         theta_index = 3
        
#     if self.is_main_fitler:
#         x_hat[my_id * num_ownship_states + 6] = u[1,0] # depth speed
#         x_hat[my_id * num_ownship_states + speed_index] = u[u_speed_index,0] # speed
#         x_hat[my_id * num_ownship_states + angular_vel_index] = u[u_ang_vel_index,0] # angular velocity

#     G = np.eye(num_states)
#     for a in range(self.num_assets): # Loop through all assets
#         start_index = a*num_ownship_states

#         # Get this asset's control input (either actual 'u' or assume constant velocity)
#         s = x_hat[start_index + speed_index,0]
#         theta_dot = x_hat[start_index + angular_vel_index,0]
#         theta_initial = x_hat[start_index+theta_index,0]

#         # Runge Kutta approximation of the control inputs effect on assets position
#         def dynamics(t, z):
#             _x_dot = s * np.cos(z[2])
#             _y_dot = s * np.sin(z[2])
#             _theta_dot = theta_dot
#             return np.array([_x_dot, _y_dot, _theta_dot])

#         t_init, t_final = 0, 1
#         z_init = np.concatenate( (x_hat[start_index:start_index + 2,0], np.array([theta_initial])) )
#         r = integrate.RK45(dynamics, t_init, z_init, t_final)
#         while r.status == "running":
#             status = r.step()

#         x_hat[start_index:start_index+2,0] = r.y[:2]
#         x_hat[start_index+theta_index,0] = r.y[2]
        
#         # Construct this asset's part of jacobian
#         if world_dim == 2:
#             G[start_index + 2, start_index + 5] = 1
#             G[start_index, start_index + 2] = -s * np.sin(theta_initial + theta_dot/2)
#             G[start_index + 1, start_index + 2] = s * np.cos(theta_initial + theta_dot/2)
#             G[start_index, start_index + 3] = np.cos(theta_initial + theta_dot/2)
#             G[start_index+1, start_index + 3] = np.sin(theta_initial + theta_dot/2)

#             G[start_index, start_index + 5] = (-s * np.sin(theta_initial + theta_dot/2)) / 2
#             G[start_index+1, start_index + 5] =  (s*np.cos(theta_initial + theta_dot/2)) / 2
#         else: # 3D World
#             G[start_index+2, start_index+6] = 1
#             G[start_index + 3, start_index + 7] = 1 # theta and theta dot
#             # dx/dtheta, dy/dtheta
#             G[start_index, start_index + 3] = -s * np.sin(theta_initial + theta_dot/2)
#             G[start_index + 1, start_index + 3] = s * np.cos(theta_initial + theta_dot/2)
#             # dx/ds, dy/ds
#             G[start_index, start_index + 4] = np.cos(theta_initial + theta_dot/2)
#             G[start_index+1, start_index + 4] = np.sin(theta_initial + theta_dot/2)
#             # dx/dtheta_dot, dy/dtheta_dot
#             G[start_index, start_index + 7] = (-s * np.sin(theta_initial + theta_dot/2)) / 2
#             G[start_index+1, start_index + 7] =  (s*np.cos(theta_initial + theta_dot/2)) / 2
            
#             # Propagate Depth
#             x_hat[start_index+2,0] = x_hat[start_index+2,0] + x_hat[start_index+6,0]
        
#     return G