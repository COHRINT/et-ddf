#!/usr/bin/env python

import os
import sys
import yaml
import numpy as np

from helpers.config_handling import load_config
from .agent import Agent
# from src.sim import run_sim_instance
# from src.ETKF import ETKF


def run_sim_instance(conns,abs_meas_vec,delta,tau,msg_drop_prob,max_time=20,
            dt=0.1,use_adaptive_tau=True,fixed_rng=999,process_noise=False,
            sensor_noise=False):
    """
    Run one simulation instance with the specified parameters.

    Keyword arguments:

        conns           -- nested list of all agents direct connections, e.g. [[2],[1,3],[2]]
                            for a 3 agent chain                    
        abs_meas_vec    -- list of agents that have absolute positioning capabilities
                            TODO: phase out, use agent config struct  
        delta           -- event-triggering threshold, allowed values=[0,inf)  
        tau             -- covariance intersection threshold per state, allowed values=[0,inf)  
        msg_drop_prob   -- probability that a message will be dropped in transit, values=[0,1]  
        max_time        -- simulation run time [sec], default=20 sec  
        dt              -- simulation time step [sec], default=0.1 sec  
        use_adaptive_tau -- boolean flag for using covarinace intersection adaptive thresholding  
        fixed_rng       -- value to seed random number generator  
        process_noise   -- structure of process noise values for every agent for testing  
        sensor_noise    -- structure of sensor noies values for every agent for testing  

    Returns:
    
        sim_data        -- simulation data structure containing results for current sim  
    """
    
    # seed random number generator
    if type(fixed_rng) is int:
        np.random.seed(fixed_rng)

    # number of agents in sim
    N = len(conns)

    # generate true starting positions and input
    for i in range(0,N):
        x_true = np.array((i*10,0,i*10,0)).T + np.random.normal([0,0,0,0],[5,0.5,5,0.5])
        x_true_vec


    # create baseline, centralized filter
    # - generate dynamics
    # - KF()

    # create agent objects
    # for 1:N
    #   - sort out connections
    #   - generate local dynamics
    #   - local filter
    #   - generate common dynamics
    #   - common filters

    # main sim loop


    

def main(conns,conn_struct_type,abs_meas_vec,delta_values,tau_values,
            msg_drop_prob_values,max_time=20,dt=0.1,use_adaptive_tau=True,
            fixed_rng=999,process_noise=False,sensor_noise=False):
    print(conns)


if __name__ == "__main__":
    # load sim config
    load_path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                    '../config/config.yaml'))
    cfg = load_config(load_path)

    # specify path for saving data
    save_path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                    '../data/'))

    # run simulation driver
    status = main(**cfg)



