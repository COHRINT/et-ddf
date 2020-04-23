from __future__ import division
import numpy as np
from copy import deepcopy

def linear_propagation(x_hat, u, num_ownship_states, my_id, is_main_fitler=False):
    num_states = x_hat.size
    num_assets = int(num_states / num_ownship_states)
    x_hat = deepcopy(x_hat)

    A = np.eye(num_states)
    num_base_states = int(num_ownship_states / 2)
    for i in range(num_assets):
        if i != my_id:
            for d in range(num_base_states):
                A[i*num_ownship_states+d, i*num_ownship_states + num_base_states + d] = 1

    x_new = None
    if is_main_fitler:
        B = np.zeros((num_states,1))
        B[my_id*num_ownship_states,0] = u[0,0]
        B[my_id*num_ownship_states+1,0] = u[1,0]
        B[my_id*num_ownship_states+2,0] = u[2,0]
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