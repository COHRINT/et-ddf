#!/usr/bin/env python

"""
Data visualization tools for OFFSET
"""

import os
import sys
import pprint
import numpy as np
import matplotlib.pyplot as plt
# import pudb; pudb.set_trace()

from .data_handling import load_sim_data

def mse_plots(metadata,data,agent_ids):
    pass


def time_trace_plots(metadata, data, agent_ids):
    """
    Creates time trace plots for provided agent ids

    Inputs:
    
        metadata -- sim run metadata
        data -- sim results data structure
        agent_ids -- list of agent ids to plot

    Outputs:

        plots -- matplotlib plot objects
    """
    # create time vector -- common to all plots
    time_vec = np.arange(start=0,
                        stop=metadata['max_time']+2*metadata['dt'],
                        step=metadata['dt'])
    
    # for each agent generate ownship estimate error plots
    for id_ in agent_ids:

        # extract agent data to plot
        a = data['agents'][id_]
        est_data = a.local_filter.state_history
        cov_data = a.local_filter.cov_history
        truth_data = a.true_state

        # extract baseline data to plot
        b = data['baseline']
        bl_est_data = b.state_history
        # print(bl_est_data[0])
        bl_cov_data = b.cov_history

        # get agent location in agent and baseline data
        _, idx = a.get_location(a.agent_id)
        bl_idx = np.arange(a.agent_id*a.num_states,a.agent_id*a.num_states+a.num_states)

        # turn data lists of list into numpy array
        est_data_vec = np.concatenate([np.array(x[idx]) for x in est_data],axis=1)
        truth_data_vec = np.concatenate([np.expand_dims(x,axis=1) for x in truth_data],axis=1)
        var_data_vec = np.concatenate([np.expand_dims(np.diag(x[np.ix_(idx,idx)]),axis=1) for x in cov_data],axis=1)

        bl_est_data_vec = np.concatenate([np.expand_dims(np.array(x[bl_idx]),axis=1) for x in bl_est_data],axis=1)
        # print(bl_est_data_vec.shape)
        bl_var_data_vec = np.concatenate([np.expand_dims(np.diag(x[np.ix_(bl_idx,bl_idx)]),axis=1) for x in bl_cov_data],axis=1)

        # create plot
        plt.figure(id_)

        # configure pyplot for using latex
        plt.rc('text', usetex=True)
        plt.rc('font',family='serif')

        plt.grid()

        plt.plot(time_vec,(est_data_vec[2,:]-truth_data_vec[2,:]),'r')
        plt.plot(time_vec,2*np.sqrt(var_data_vec[0,:]),'r--')
        plt.plot(time_vec,-2*np.sqrt(var_data_vec[0,:]),'r--')

        # plt.plot(time_vec,(bl_est_data_vec[2,:]-truth_data_vec[2,:]),'g')
        # plt.plot(time_vec,2*np.sqrt(bl_var_data_vec[0,:]),'g--')
        # plt.plot(time_vec,-2*np.sqrt(bl_var_data_vec[0,:]),'g--')

        plt.xlabel('Time [s]')
        plt.ylabel('Est error [m]')
        plt.title(r'Agent {} ownship $\xi$ est. err: $\delta={}$, $\tau_g={}$, msg drop={}'.format(id_+1,metadata['delta'],metadata['tau_goal'],metadata['msg_drop_prob']))
        plt.legend(['Est error',r'$\pm 2\sigma$','','bl est error',r'$\pm 2\sigma$',''])

    plt.show()

def test_mse_plots():

    save_path = '../../data/sim_20190417-122247.pckl'
    data = load_sim_data(save_path)

    # pprint.pprint(data)
    time_trace_plots(data['results'][0]['metadata'],
                data['results'][0]['results'],
                [0,14,22])

if __name__ == "__main__":

    test_mse_plots()