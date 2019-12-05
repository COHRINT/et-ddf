#!/usr/bin/env python

"""
Data visualization tools for ET-DDF and QET-DDF
"""

import os
import sys
import pprint
import numpy as np
import matplotlib.pyplot as plt
import argparse
import pudb; pudb.set_trace()

from etddf.helpers.data_handling import load_sim_data, load_metadata

# def print_data_usage(path,agent_ids):
#     """
#     Print information about data transfer, including covariance intersection triggers and message sending rates.
#     """

TITLE_SIZE = 16
LABEL_SIZE = 14
TICK_SIZE = 12

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
    # figs = [['delta10','drop00','tau5'],['delta10','drop00','tau7'],['delta20','drop00','tau5'],['delta20','drop00','tau7']]
    # figs = [['drop00','delta15','tau5'],['drop02','delta20','tau5']]
    # figs = [['delta10','drop00','tau5'],['delta10','drop00','tau7'],['delta20','drop00','tau5'],['delta20','drop00','tau7']]#,
    #         ['delta05','drop00','tau5'],['delta05','drop00','tau7'],['delta15','drop00','tau5'],['delta15','drop00','tau7'],
    #         ['delta05','drop00','tau35'],['delta10','drop00','tau35'],['delta15','drop00','tau35'],['delta20','drop00','tau35']]
    # figs = [['delta10','drop00','tau0'],['delta10','drop00','tau05'],['delta20','drop00','tau0'],['delta20','drop00','tau05'],
    #         ['delta05','drop00','tau0'],['delta05','drop00','tau05'],['delta15','drop00','tau0'],['delta15','drop00','tau05'],
    #         ['delta05','drop00','tau1'],['delta10','drop00','tau1'],['delta15','drop00','tau1'],['delta20','drop00','tau1']]
    # figs = [['delta05','drop00','tau15'],['delta10','drop00','tau15'],['delta15','drop00','tau15'],['delta20','drop00','tau15']]
    # figs = [['delta05','drop00','tau20'],['delta10','drop00','tau20'],['delta15','drop00','tau20'],['delta20','drop00','tau20'],
    #         ['delta05','drop00','tau25'],['delta10','drop00','tau25'],['delta15','drop00','tau25'],['delta20','drop00','tau25'],
    # figs = [['delta05','drop00','tau30'],['delta10','drop00','tau30'],['delta15','drop00','tau30'],['delta20','drop00','tau30']]
    # figs = [['delta15','drop00','tau30']]
    # figs = [['delta15','drop00','tau70']]
    figs = [['delta10','drop00','tau35'],['delta10','drop00','tau5']]

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
            data.append(load_sim_data(os.path.join(os.path.abspath(path),file)))
            
        # create time vector -- common to all plots
        time_vec = np.arange(start=0,
                            stop=metadata['max_time']+metadata['dt'],
                            step=metadata['dt'])

        # create figure for figure parameter set
        plt.figure()
        legend_str = []
        # create params title
        delta_str = fig[0].split('delta')[1]
        if int(delta_str) > 9:
            delta_str = str(int(delta_str)/10)
        tau_str = fig[2].split('tau')[1]
        if int(tau_str) > 9:
            tau_str = str(int(tau_str)/10)
        params_str = r'$\delta$=' + delta_str + r', $\tau$=' + tau_str

        # counter for color picking
        color_cnt = 0

        # configure pyplot for using latex
        plt.rc('text', usetex=True)
        plt.rc('font',family='serif')
        plt.grid(True)
        plt.title('Position RMSE, ' + params_str + ', GPS agents: ' + str(metadata['agent_cfg']['sensors']['lin_abs_pos']['agents']),size=TITLE_SIZE)
        plt.xlabel('Time [s]',size=LABEL_SIZE)
        plt.ylabel(r'Est error [$m$]',size=LABEL_SIZE)

        for label in (plt.gca().get_xticklabels() + plt.gca().get_yticklabels()):
            label.set_fontsize(TICK_SIZE)

        # for each loaded data file
        for param_data in data:
            # for each agent generate mse plot
            baseline_mse_data = param_data['results']['baseline_mse']
            baseline_mse_data_avg = np.mean(baseline_mse_data,axis=1)
            plt.plot(time_vec,np.sqrt(baseline_mse_data_avg),'--',color='C7')
            legend_str.append('baseline avg')

            for id_ in agent_ids:

                # extract agent data to plot
                mse_data = param_data['results']['mse'][:,id_]
                # baseline_mse_data = param_data['results']['baseline_mse'][:,id_]
                # baseline_mse_data_avg = np.mean(baseline_mse_data,axis=1)

                mse_std_data = np.mean(param_data['results']['mse_std'][:,id_])
                print('Agent {} RMSE avg std: {}'.format(id_,np.sqrt(mse_std_data)))

                color_str = 'C' + str(color_cnt%12)

                plt.plot(time_vec,np.sqrt(mse_data),color=color_str)
                # plt.plot(time_vec,np.sqrt(baseline_mse_data),'--',color=color_str)
                # plt.plot(time_vec,np.sqrt(baseline_mse_data_avg),'--',color='C7')

                legend_str.append('{}'.format(id_))
                # legend_str.append('{} baseline'.format(id_))
                # legend_str.append('baseline'.format(id_))

                # plt.title(r'Agent {} ownship pos MSE: $\delta={}$, $\tau_g={}$, msg drop={}'.format(id_+1,param_data['metadata']['delta_value'],param_data['metadata']['tau_value'],param_data['metadata']['msg_drop_prob_value']))

                color_cnt += 1

            # legend_str.append(r'$\delta={}$'.format(param_data['metadata']['delta_value']))

            print('-----')
            print(str(fig) + ', Abs. pos. -- ' + str(metadata['agent_cfg']['sensors']['lin_abs_pos']['agents']))
            print('Total possible messages to send: {}'.format(param_data['results']['msgs_total']))
            print('Total messages sent: {}'.format(param_data['results']['msgs_sent']))
            print('CI triggers: {}'.format(param_data['results']['ci_total']))
            print('CI trigger rate: {}'.format(param_data['results']['ci_rate']))

        plt.legend(legend_str,loc='upper left')
        plt.ylim([-1,40])

    # plt.show()

def rel_mse_plots(path,agent_ids):
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
    # figs = [['delta10','drop00','tau5'],['delta10','drop00','tau7'],['delta20','drop00','tau5'],['delta20','drop00','tau7']]
    # figs = [['drop00','delta15','tau5'],['drop02','delta20','tau5']]
    # figs = [['delta10','drop00','tau5'],['delta10','drop00','tau7'],['delta20','drop00','tau5'],['delta20','drop00','tau7']]#,
    #         ['delta05','drop00','tau5'],['delta05','drop00','tau7'],['delta15','drop00','tau5'],['delta15','drop00','tau7'],
    #         ['delta05','drop00','tau35'],['delta10','drop00','tau35'],['delta15','drop00','tau35'],['delta20','drop00','tau35']]
    # figs = [['delta10','drop00','tau0'],['delta10','drop00','tau05'],['delta20','drop00','tau0'],['delta20','drop00','tau05'],
    #         ['delta05','drop00','tau0'],['delta05','drop00','tau05'],['delta15','drop00','tau0'],['delta15','drop00','tau05'],
    #         ['delta05','drop00','tau1'],['delta10','drop00','tau1'],['delta15','drop00','tau1'],['delta20','drop00','tau1']]
    # figs = [['delta05','drop00','tau15'],['delta10','drop00','tau15'],['delta15','drop00','tau15'],['delta20','drop00','tau15']]
    # figs = [['delta05','drop00','tau20'],['delta10','drop00','tau20'],['delta15','drop00','tau20'],['delta20','drop00','tau20'],
    #         ['delta05','drop00','tau25'],['delta10','drop00','tau25'],['delta15','drop00','tau25'],['delta20','drop00','tau25'],
    # figs = [['delta05','drop00','tau30'],['delta10','drop00','tau30'],['delta15','drop00','tau30'],['delta20','drop00','tau30']]
    # figs = [['delta15','drop00','tau30']]
    # figs = [['delta15','drop00','tau70']]
    figs = [['delta10','drop00','tau35'],['delta10','drop00','tau5']]

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
            data.append(load_sim_data(os.path.join(os.path.abspath(path),file)))
            
        # create time vector -- common to all plots
        time_vec = np.arange(start=0,
                            stop=metadata['max_time']+metadata['dt'],
                            step=metadata['dt'])

        # create figure for figure parameter set
        plt.figure()
        legend_str = []
        # create params title
        delta_str = fig[0].split('delta')[1]
        if int(delta_str) > 9:
            delta_str = str(int(delta_str)/10)
        tau_str = fig[2].split('tau')[1]
        if int(tau_str) > 9:
            tau_str = str(int(tau_str)/10)
        params_str = r'$\delta$=' + delta_str + r', $\tau$=' + tau_str

        # color_cnt = 0

        # configure pyplot for using latex
        plt.rc('text', usetex=True)
        plt.rc('font',family='serif')
        plt.grid(True)
        plt.title('Rel. Position RMSE, ' + params_str + ', GPS agents: ' + str(metadata['agent_cfg']['sensors']['lin_abs_pos']['agents']),size=TITLE_SIZE)
        plt.xlabel('Time [s]',size=LABEL_SIZE)
        plt.ylabel(r'Est error [$m$]',size=LABEL_SIZE)

        for label in (plt.gca().get_xticklabels() + plt.gca().get_yticklabels()):
            label.set_fontsize(TICK_SIZE)

        # for each loaded data file
        for param_data in data:
            # for each agent generate mse plot
            color_cnt = 0
            for id_ in agent_ids:
                # extract agent data to plot
                rel_mse_data = param_data['results']['rel_mse'][id_]
                # baseline_mse_data = param_data['results']['baseline_mse'][:,id_]

                for conn in range(0,rel_mse_data.shape[1]):
                    color_str = 'C' + str(color_cnt%10)

                    plt.plot(time_vec,np.sqrt(rel_mse_data[:,conn]),color=color_str)

                    legend_str.append(r'${}\rightarrow {}$'.format(id_,metadata['agent_cfg']['conns'][id_][conn]))
                    color_cnt += 1


                # plt.plot(time_vec,np.sqrt(baseline_mse_data),'--',color=color_str)

                # plt.title(r'Agent {} ownship pos MSE: $\delta={}$, $\tau_g={}$, msg drop={}'.format(id_+1,param_data['metadata']['delta_value'],param_data['metadata']['tau_value'],param_data['metadata']['msg_drop_prob_value']))

            # legend_str.append(r'$\delta={}$'.format(param_data['metadata']['delta_value']))
                # legend_str.append('{}'.format(id_))

            print('-----')
            print(str(fig) + ', Abs. pos. -- ' + str(metadata['agent_cfg']['sensors']['lin_abs_pos']['agents']))
            print('Total possible messages to send: {}'.format(param_data['results']['msgs_total']))
            print('Total messages sent: {}'.format(param_data['results']['msgs_sent']))
            print('CI triggers: {}'.format(param_data['results']['ci_total']))
            print('CI trigger rate: {}'.format(param_data['results']['ci_rate']))

        plt.legend(legend_str,loc='upper left')
        plt.ylim([-1,10])

    # plt.show()

def time_trace_plots(path, agent_ids):
    """
    Creates time trace plots for provided agent ids

    Inputs:
    
        metadata -- sim run metadata
        data -- sim results data structure
        agent_ids -- list of agent ids to plot

    Outputs:

        plots -- matplotlib plot objects
    """
    # list of params for figures --> params not specified will have all values plotted
    # figs = [['delta10','drop00','tau5'],['delta10','drop00','tau7'],['delta20','drop00','tau5'],['delta20','drop00','tau7']]
    # figs = [['drop00','delta15','tau5'],['drop02','delta20','tau5']]
    # figs = [['delta10','drop00','tau5'],['delta10','drop00','tau7'],['delta20','drop00','tau5'],['delta20','drop00','tau7']]#,
    #         ['delta05','drop00','tau5'],['delta05','drop00','tau7'],['delta15','drop00','tau5'],['delta15','drop00','tau7'],
    #         ['delta05','drop00','tau35'],['delta10','drop00','tau35'],['delta15','drop00','tau35'],['delta20','drop00','tau35']]
    # figs = [['delta10','drop00','tau0'],['delta10','drop00','tau05'],['delta20','drop00','tau0'],['delta20','drop00','tau05'],
    #         ['delta05','drop00','tau0'],['delta05','drop00','tau05'],['delta15','drop00','tau0'],['delta15','drop00','tau05'],
    #         ['delta05','drop00','tau1'],['delta10','drop00','tau1'],['delta15','drop00','tau1'],['delta20','drop00','tau1']]
    # figs = [['delta05','drop00','tau15'],['delta10','drop00','tau15'],['delta15','drop00','tau15'],['delta20','drop00','tau15']]
    # figs = [['delta05','drop00','tau20'],['delta10','drop00','tau20'],['delta15','drop00','tau20'],['delta20','drop00','tau20'],
    #         ['delta05','drop00','tau25'],['delta10','drop00','tau25'],['delta15','drop00','tau25'],['delta20','drop00','tau25'],
    # figs = [['delta05','drop00','tau30'],['delta10','drop00','tau30'],['delta15','drop00','tau30'],['delta20','drop00','tau30']]
    # figs = [['delta15','drop00','tau30']]
    # figs = [['delta15','drop00','tau70']]
    figs = [['delta10','drop00','tau35'],['delta10','drop00','tau5']]

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
            data.append(load_sim_data(os.path.abspath(os.path.join(path,file))))
            
        # create time vector -- common to all plots
        time_vec = np.arange(start=0,
                            stop=metadata['max_time']+metadata['dt'],
                            step=metadata['dt'])
    
        # create figure for figure parameter set
        # plt.figure()
        legend_str = []
        # create params title
        delta_str = fig[0].split('delta')[1]
        if int(delta_str) > 9:
            delta_str = str(int(delta_str)/10)
        tau_str = fig[2].split('tau')[1]
        if int(tau_str) > 9:
            tau_str = str(int(tau_str)/10)
        params_str = r'$\delta$=' + delta_str + r', $\tau$=' + tau_str

        # configure pyplot for using latex
        plt.rc('text', usetex=True)
        plt.rc('font',family='serif')
        # plt.grid(True)
        # plt.title(str(fig) + ', Abs. pos. -- ' + str(metadata['agent_cfg']['sensors']['lin_abs_pos']['agents']))
        # plt.xlabel('Time [s]')
        # plt.ylabel(r'Est error [$m$]')
        
        # for each loaded data file
        for param_data in data:
            # for each agent generate mse plot
            for id_ in agent_ids:

                # extract agent data to plot
                # mse_data = param_data['results']['etddf_mse'][:,id_]
                # nav_mse_data = param_data['results']['nav_mse'][:,id_]
                etddf_state_error = param_data['results']['state_error']
                etddf_cov_history = param_data['results']['cov_error']

                # print(etddf_state_error[id_].shape)
                # print(etddf_state_error[id_][7,:,:])
                # print(nav_cov_history[0].shape)
                # print(etddf_cov_history[0].shape)
                # # assert(False)

                # Position estimate error
                plt.figure()
                plt.subplot(311)
                plt.grid(True)
                plt.plot(time_vec,etddf_state_error[id_][:,0],'C0')
                plt.fill_between(time_vec,-2*np.sqrt(etddf_cov_history[id_][:,0,0]),2*np.sqrt(etddf_cov_history[id_][:,0,0]),alpha=0.1,color='C0')
                plt.legend(['est error',r'$\pm 2 \sigma$'])
                plt.ylabel(r'N [$m$]',size=LABEL_SIZE)
                plt.title('Agent ' + str(id_) + ', ' + params_str + ', GPS agents: ' + str(metadata['agent_cfg']['sensors']['lin_abs_pos']['agents']) + ', Position error',size=TITLE_SIZE)
                plt.ylim([-25,25])

                for label in (plt.gca().get_xticklabels() + plt.gca().get_yticklabels()):
                    label.set_fontsize(TICK_SIZE-1)

                plt.subplot(312)
                plt.grid(True)
                plt.plot(time_vec,etddf_state_error[id_][:,2])
                plt.fill_between(time_vec,-2*np.sqrt(etddf_cov_history[id_][:,2,2]),2*np.sqrt(etddf_cov_history[id_][:,2,2]),alpha=0.1,color='C0')
                # plt.legend(['etddf',r'etddf $\pm 2 \sigma$','nav',r'nav $\pm 2 \sigma$'])
                # plt.ylabel(r'Est error E [$m$]',size=LABEL_SIZE)
                plt.ylabel(r'E [$m$]',size=LABEL_SIZE)
                plt.ylim([-25,25])

                for label in (plt.gca().get_xticklabels() + plt.gca().get_yticklabels()):
                    label.set_fontsize(TICK_SIZE-1)

                plt.subplot(313)
                plt.grid(True)
                plt.plot(time_vec,etddf_state_error[id_][:,4])
                plt.fill_between(time_vec,-2*np.sqrt(etddf_cov_history[id_][:,4,4]),2*np.sqrt(etddf_cov_history[id_][:,4,4]),alpha=0.1,color='C0')
                # plt.legend(['etddf',r'etddf $\pm 2 \sigma$','nav',r'nav $\pm 2 \sigma$'])
                plt.xlabel('Time [s]',size=LABEL_SIZE)
                plt.ylabel(r'D[$m$]',size=LABEL_SIZE)
                plt.ylim([-25,25])

                for label in (plt.gca().get_xticklabels() + plt.gca().get_yticklabels()):
                    label.set_fontsize(TICK_SIZE-1)

                # Velocity estimate error
                plt.figure()
                plt.subplot(311)
                plt.grid(True)
                plt.plot(time_vec,etddf_state_error[id_][:,1],'C0')
                plt.fill_between(time_vec,-2*np.sqrt(etddf_cov_history[id_][:,1,1]),2*np.sqrt(etddf_cov_history[id_][:,1,1]),alpha=0.1,color='C0')
                plt.legend(['est error',r'$\pm 2 \sigma$'])
                plt.ylabel(r'N [$m/s$]',size=LABEL_SIZE)
                plt.title('Agent ' + str(id_) + ', ' + params_str + ', GPS agents: ' + str(metadata['agent_cfg']['sensors']['lin_abs_pos']['agents']) + ', Velocity error',size=TITLE_SIZE)
                plt.ylim([-7,7])

                for label in (plt.gca().get_xticklabels() + plt.gca().get_yticklabels()):
                    label.set_fontsize(TICK_SIZE-1)

                plt.subplot(312)
                plt.grid(True)
                plt.plot(time_vec,etddf_state_error[id_][:,3])
                plt.fill_between(time_vec,-2*np.sqrt(etddf_cov_history[id_][:,3,3]),2*np.sqrt(etddf_cov_history[id_][:,3,3]),alpha=0.1,color='C0')
                # plt.legend(['etddf',r'etddf $\pm 2 \sigma$','nav',r'nav $\pm 2 \sigma$'])
                plt.ylabel(r'E [$m/s$]',size=LABEL_SIZE)
                plt.ylim([-7,7])

                for label in (plt.gca().get_xticklabels() + plt.gca().get_yticklabels()):
                    label.set_fontsize(TICK_SIZE-1)

                plt.subplot(313)
                plt.grid(True)
                plt.plot(time_vec,etddf_state_error[id_][:,5])
                plt.fill_between(time_vec,-2*np.sqrt(etddf_cov_history[id_][:,5,5]),2*np.sqrt(etddf_cov_history[id_][:,5,5]),alpha=0.1,color='C0')
                # plt.legend(['etddf',r'etddf $\pm 2 \sigma$','nav',r'nav $\pm 2 \sigma$'])
                plt.xlabel('Time [s]',size=LABEL_SIZE)
                plt.ylabel(r'D[$m$]',size=LABEL_SIZE)
                plt.ylim([-7,7])

                for label in (plt.gca().get_xticklabels() + plt.gca().get_yticklabels()):
                    label.set_fontsize(TICK_SIZE-1)

                # plt.title(r'Agent {} ownship pos MSE: $\delta={}$, $\tau_g={}$, msg drop={}'.format(id_+1,param_data['metadata']['delta_value'],param_data['metadata']['tau_value'],param_data['metadata']['msg_drop_prob_value']))

            # legend_str.append(r'$\delta={}$'.format(param_data['metadata']['delta_value']))
                # legend_str.append('{} etddf'.format(id_))
                # legend_str.append('{} nav'.format(id_))

            # print('-----')
            # print(str(fig) + ', Abs. pos. -- ' + str(metadata['agent_cfg']['sensors']['lin_abs_pos']['agents']))
            # print('Total possible messages to send: {}'.format(param_data['results']['msgs_total']))
            # print('Total messages sent: {}'.format(param_data['results']['msgs_sent']))
            # print('CI triggers: {}'.format(param_data['results']['ci_total']))
            # print('CI trigger rate: {}'.format(param_data['results']['ci_rate']))

        # plt.legend(legend_str)

    # plt.show()

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
    parser.add_argument('-u','--data-usage',dest='data_usage',action='store_true',
                    help='Print information about data usage.')
    parser.add_argument('-f','--file-path',type=str,dest='file_path',action='store',
                    help='specify file path of sim data')
    parser.add_argument('-d','--dir-path',type=str,dest='dir_path',action='store',
                    help='specify path to sim data directory')
    parser.add_argument('-r','--rel-mse',dest='rel_mse_flag',action='store_true',
                    help='plot relative mean-squared-errors (relMSE)')
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
    if args.tt_flag:
        time_trace_plots(args.dir_path, agents)

    if args.mse_flag:
        mse_plots(args.dir_path, agents)

    if args.rel_mse_flag:
        rel_mse_plots(args.dir_path, agents)

    plt.show()
    # if args.data_usage:
        # print_data_usage(args.dir_path, agents)