from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from pdb import set_trace
from copy import deepcopy

def g_func(x_hat, u, my_id, num_ownship_states):
    x_state = deepcopy(x_hat) # editing values through function pointers gets strange in Python
    print("Input for asset " + str(my_id) + ":")
    print(x_state)
    print(u)
    u_input = deepcopy(u)
    x_state[my_id * num_ownship_states + 3] = u_input[0,0] # speed
    x_state[my_id * num_ownship_states + 5] = u_input[1,0] # angular velocity

    G = np.zeros((num_states, num_states))
    for a in range(num_assets):
        start_index = a*num_ownship_states
        s = x_state[start_index + 3,0]
        theta_dot = x_state[start_index + 5,0]

        theta_initial = x_state[start_index+2,0]
        def dynamics(t, z):
            _x_dot = s * np.cos(z[2])
            _y_dot = s * np.sin(z[2])
            _theta_dot = theta_dot
            return np.array([_x_dot, _y_dot, _theta_dot])

        t_init, t_final = 0, 1
        z_init = x_state[start_index:start_index + 3,0]
        r = integrate.RK45(dynamics, t_init, z_init, t_final)
        while r.status == "running":
            status = r.step()

        x_state[start_index: start_index+3,0] = r.y
        
        # Construct this asset's part of jacobian
        G[start_index,start_index] = 1
        G[start_index + 1,start_index + 1] = 1
        G[start_index + 2, start_index + 2] = 1
        G[start_index, start_index + 2] = -s * np.sin(theta_initial + theta_dot/2)
        G[start_index + 1, start_index + 2] = s * np.cos(theta_initial + theta_dot/2)
        # G[start_index, start_index + 2] = -s * np.sin(theta_initial)
        # G[start_index + 1, start_index + 2] = s * np.cos(theta_initial)
        G[start_index + 4, start_index + 4] = 1

        if a != my_id:
            G[start_index + 3, start_index + 3] = 1
            G[start_index + 5, start_index + 5] = 1

    print("Output")
    print(x_state)
    return (x_state, G)

def linear_propagation(x, u, world_dim, num_ownship_states, my_id=0):
    x = deepcopy(x).reshape(-1,1)
    u = deepcopy(u).reshape(-1,1)
    num_states = x.size
    num_assets = int(num_states / num_ownship_states)
    if world_dim == 1 and num_ownship_states == 1:
        A = B = np.eye(1)
        x = A*x + B*u
        return x
    elif world_dim == 1 and num_ownship_states == 2:
        A = np.array([[1,0],[0,0]])
        B = np.array([[1],[1]])
        x_new = np.dot(A,x) + np.dot(B, u)
        return x_new
    elif world_dim == 2 and num_ownship_states == 4:
        A = np.eye(num_states)
        for i in range(num_assets):
            if i == my_id:
                A[i*num_ownship_states+2,i*num_ownship_states+2] = 0 # set velocity to zero
                A[i*num_ownship_states+3,i*num_ownship_states+3] = 0 # set velocity to zero
            else:
                A[i*num_ownship_states,i*num_ownship_states+2] = 1
                A[i*num_ownship_states+1,i*num_ownship_states+3] = 1
        B = np.zeros((num_states,u.size))
        B[my_id*num_ownship_states,0] = 1
        B[my_id*num_ownship_states+1,0] = 1
        B[my_id*num_ownship_states+2,1] = 1
        B[my_id*num_ownship_states+3,1] = 1
        x = np.dot(A,x) + np.dot(B,u)
        return x
    else:
        raise NotImplementedError("Linear Propagation Not defined for state configuration")