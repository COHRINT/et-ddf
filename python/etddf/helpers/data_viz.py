#!/usr/bin/env python

"""
Data visualization tools for OFFSET
"""

import os
import sys
import pprint
import numpy as np
import matplotlib.pyplot as plt
import argparse
# import pudb; pudb.set_trace()

from etddf.helpers.data_handling import load_sim_data, load_metadata

def mse_plots(path,agent_ids):
    """
    Creates mean-squared-error plors for provided agent ids.

    Inputs:

        metadata -- sim run metadata
        data -- sim results data structure
        agent_ids -- list of agent ids to plot

    Outputs:

        plots -- matplotlib plot objects
    """
    # list of params for figures --> params not specified will have all values plotted
    figs = [['delta10','drop00','tau5'],['delta10','drop00','tau7'],['delta20','drop00','tau5'],['delta20','drop00','tau7']]
    # figs = [['drop00','delta15','tau5'],['drop02','delta20','tau5']]

    # load simulation metadata and get ids of agents to plot
    metadata = load_metadata(path)['cfg']
    if len(agent_ids) == 1 and agent_ids[0] == -1:
        agent_ids = list(range(0,len(metadata['agent_cfg']['conns'])))

    # for each fig to be created, get data
    for fig in figs:

        # get all sim data files with desired params
        all_files = os.listdir(path)
        files_to_load = []
        for file in all_files:
            keep_flag = True
            for param in fig:
                if param not in file:
                    keep_flag = False
            if keep_flag: files_to_load.append(file)
        
        data = []
        for file in files_to_load:
            data.append(load_sim_data(os.path.join(path,file)))
            
        # create time vector -- common to all plots
        time_vec = np.arange(start=0,
                            stop=metadata['max_time']+metadata['dt'],
                            step=metadata['dt'])

        # create figure for figure parameter set
        plt.figure()
        legend_str = []

        # configure pyplot for using latex
        plt.rc('text', usetex=True)
        plt.rc('font',family='serif')
        plt.grid(True)
        plt.xlabel('Time [s]')
        plt.ylabel(r'Est error [$m^2$]')

        # for each loaded data file
        for param_data in data:
            # for each agent generate mse plot
            for id_ in agent_ids:

                # extract agent data to plot
                mse_data = param_data['results']['mse'][:,id_]
                plt.plot(time_vec,mse_data)

                # plt.title(r'Agent {} ownship pos MSE: $\delta={}$, $\tau_g={}$, msg drop={}'.format(id_+1,param_data['metadata']['delta_value'],param_data['metadata']['tau_value'],param_data['metadata']['msg_drop_prob_value']))

            legend_str.append(r'$\delta={}$'.format(param_data['metadata']['delta_value']))

        plt.legend(legend_str)

    plt.show()

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
    dim = 2

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
        bl_cov_data = b.cov_history

        # get agent location in agent and baseline data
        _, idx = a.get_location(a.agent_id)
        bl_idx = np.arange(a.agent_id*a.num_states,a.agent_id*a.num_states+a.num_states)

        # turn data lists of list into numpy array
        est_data_vec = np.concatenate([np.array(x[idx]) for x in est_data],axis=1)
        truth_data_vec = np.concatenate([np.expand_dims(x,axis=1) for x in truth_data],axis=1)
        var_data_vec = np.concatenate([np.expand_dims(np.diag(x[np.ix_(idx,idx)]),axis=1) for x in cov_data],axis=1)

        bl_est_data_vec = np.concatenate([np.expand_dims(np.array(x[bl_idx]),axis=1) for x in bl_est_data],axis=1)
        bl_var_data_vec = np.concatenate([np.expand_dims(np.diag(x[np.ix_(bl_idx,bl_idx)]),axis=1) for x in bl_cov_data],axis=1)

        # create plot
        plt.figure(id_)

        # configure pyplot for using latex
        plt.rc('text', usetex=True)
        plt.rc('font',family='serif')

        plt.grid()

        plt.plot(time_vec,(est_data_vec[dim,:]-truth_data_vec[dim,:]),'r')
        plt.plot(time_vec,2*np.sqrt(var_data_vec[dim,:]),'r--')
        plt.plot(time_vec,-2*np.sqrt(var_data_vec[dim,:]),'r--')

        plt.plot(time_vec,(bl_est_data_vec[dim,:]-truth_data_vec[dim,:]),'g')
        # plt.plot(time_vec,2*np.sqrt(bl_var_data_vec[dim,:]),'g--')
        # plt.plot(time_vec,-2*np.sqrt(bl_var_data_vec[dim,:]),'g--')

        plt.xlabel('Time [s]')
        plt.ylabel('Est error [m]')
        plt.title(r'Agent {} ownship $\xi$ est. err: $\delta={}$, $\tau_g={}$, msg drop={}'.format(id_+1,metadata['delta'],metadata['tau_goal'],metadata['msg_drop_prob']))
        plt.legend(['Est error',r'$\pm 2\sigma$','','BL est error',r'$\pm 2\sigma$',''])

    plt.show()

def test_mse_plots():

    save_path = '../../data/sim_20190418-010908.pckl'
    data = load_sim_data(save_path)

    # pprint.pprint(data)
    time_trace_plots(data['results'][0]['metadata'],
                data['results'][0]['results'],
                [0,2,4])

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Plot simulation results from simulation data structure, stored as pickle file.')
    parser.add_argument('agents',metavar='A',type=int,action='store',nargs='+',
                    help='ids of agents to plot (use -1 for all agents)')
    parser.add_argument('-t','--time-trace',dest='tt_flag',action='store_true',
                    help='plot time traces of estimate error')
    parser.add_argument('-m','--mse',dest='mse_flag',action='store_true',
                    help='plot mean-squared-errors (MSE)')
    parser.add_argument('-f','--file-path',type=str,dest='file_path',action='store',
                    help='specify file path of sim data')
    parser.add_argument('-d','--dir-path',type=str,dest='dir_path',action='store',
                    help='specify path to sim data directory')
    args = parser.parse_args()

    # TODO: add arg for local, common, or both
    # TODO: figure out how to return plot objects and show after tt and mse plotting
    # TODO: default to plotting most recently save pickle file, instead of hardcoded path

    # set data path
    if args.file_path is None:
        save_path = '../../data/sim_20190418-010908.pckl'
    else:
        save_path = args.file_path

    # load data
    # data = load_sim_data(save_path)

    # get all agent ids if param is all agents (-1)
    agents = args.agents
    # if len(args.agents) == 1 and args.agents[0] == -1:
        # agents = list(range(0,data['results'][0]['metadata']['num_agents']))

    # generate plots
    # if args.tt_flag:
    #     time_trace_plots(data['results'][0]['metadata'],
    #             data['results'][0]['results'],
    #             agents)

    if args.mse_flag:
        mse_plots(args.dir_path, agents)