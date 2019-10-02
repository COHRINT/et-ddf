#!/usr/bin/env python

"""
Simulation instance class implementation.
"""

import os
import sys
import yaml
import numpy as np
import scipy.linalg
import pudb; # pudb.set_trace()
import argparse
from copy import deepcopy
import scipy.io

from etddf.agent import Agent
from etddf.filters.kf import KF
from etddf.filters.etkf import ETKF
from etddf.dynamics import *
from etddf.helpers.config_handling import load_config
from etddf.helpers.msg_handling import MeasurementMsg, StateMsg
from etddf.helpers.data_handling import package_results, save_sim_data, make_data_directory, write_metadata_file
from etddf.helpers.data_viz import mse_plots, time_trace_plots
from etddf.quantization import Quantizer, covar_diagonalize

class SimInstance(object):
    """
    Run one simulation instance with the specified parameters.

    Keyword arguments:
 
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

    def __init__(self,delta,tau,msg_drop_prob,baseline_cfg,
                    agent_cfg,max_time=20,dt=0.1,use_adaptive_tau=True,
                    fixed_rng=None,process_noise=False,sensor_noise=False,
                    quantization_flag=False,diagonalization_flag=False):

        self.max_time = max_time
        self.dt = dt
        self.sim_time = 0
        self.sim_time_step = 0
        self.num_agents = len(agent_cfg['conns'])

        self.connections = agent_cfg['conns']

        self.dynamics = agent_cfg['dynamics_fxn']
        self.sensors = agent_cfg['sensors']

        self.delta = delta
        self.tau_state_goal = tau
        self.tau = tau*0.75
        self.msg_drop_prob = msg_drop_prob
        self.use_adaptive_tau = use_adaptive_tau

        # data compression flags and object
        self.quantization = quantization_flag
        self.diagonalization = diagonalization_flag
        if self.quantization:
            self.quantizer = Quantizer(quantizer_fxn='x2')

        self.fixed_rng = fixed_rng
        if self.fixed_rng is not False:
            np.random.seed(self.fixed_rng)
            print('Fixing random number generator with seed: {}'.format(self.fixed_rng))


        # load matlab data for verification
        # self.sensor_noise_data = scipy.io.loadmat('../matlab/sensor_data_6agents.mat')
        # self.x_start_data = self.sensor_noise_data['x_start_data']
        # self.w_data = self.sensor_noise_data['w_data']
        # self.v_data = self.sensor_noise_data['v_data']
        # self.v_rel_data = self.sensor_noise_data['v_rel_data']

        self.all_msgs = []

        # generate ground truth starting positions
        x_true_vec = self.init_ground_truth()

        # create baseline filter and agetns from config
        self.baseline_filter = self.init_baseline(x_true_vec,
                                                    baseline_cfg['dynamics_fxn'],
                                                    {},
                                                    baseline_cfg['sensors'])
        self.agents = self.init_agents(x_true_vec,
                                        agent_cfg['dynamics_fxn'],
                                        {},
                                        agent_cfg['sensors'])
        
        print('Initialized baseline and agents')

    def init_ground_truth(self):
        """
        Create ground truth initial position for each agent
        """
        x_true_vec = np.zeros(6*self.num_agents)
        for i in range(0,self.num_agents):
            start_noise = np.random.normal([0,0,0,0,0,0],[5,0.5,5,0.5,5,0.5])
            # start_noise = self.x_start_data[0,i]
            x_true = np.array(((i+1)*10,0,(i+1)*10,0,5,0)).T + start_noise
            x_true_vec[6*i:6*i+6] = x_true

        return x_true_vec

    def init_baseline(self,x_true_vec,dynamics_fxn='lin_ncv',dynamics_fxn_params={},sensors={}):
        """
        Create baseline centralized filter for comparison.

        Inputs:

            x_true_vec  -- initial true starting positions for every agent
            dynamics    -- name of dynamics fxn to be used
            sensors     -- dictionary of sensors and associated parameters

        Returns:

            baseline_filter -- instance of baseline centralized filter
        """
        # get sensor noise params
        R_abs = sensors['lin_abs_pos']['noise']
        R_rel = sensors['lin_rel_range']['noise']

        # get dynamics and process noise matrices
        F_full, G_full, Q_full = globals()[dynamics_fxn](self.dt,self.num_agents)

        # create initial estimate w/ ground truth
        x0_full = x_true_vec
        # create initial covariance
        P0_full = 100*np.eye(6*self.num_agents)

        # create filter instance
        baseline_filter = KF(F_full,G_full,0,0,Q_full,R_abs,R_rel,x0_full,P0_full,0)

        return baseline_filter

    def init_agents(self,x_true_vec,dynamics_fxn='lin_ncv',dynamics_fxn_params={},sensors={}):
        """
        Create agents, including associated local filters and common information filters.

        Inputs:

            x_true_vec  -- initial true starting positions for every agent
            dynamics    -- name of dynamics fxn to be used
            sensors     -- dictionary of sensors and associated parameters

        Returns:

            agents -- list of Agent instances
        """
        R_abs = sensors['lin_abs_pos']['noise']
        R_rel = sensors['lin_rel_range']['noise']

        # initialize list of agents
        agents = []

        for i in range(0,self.num_agents):

            agent_id = deepcopy(i)
            ids = sorted(deepcopy(self.connections[agent_id]))
            ids.append(agent_id)

            # build list of distance one and distance two neighbors for each agent
            # each agent gets full list of connections
            neighbor_conn_ids = []
            for j in range(0,len(self.connections[agent_id])):
                for k in range(0,len(self.connections[self.connections[agent_id][j]])):
                    if not self.connections[self.connections[agent_id][j]][k] in neighbor_conn_ids:
                        neighbor_conn_ids += self.connections[self.connections[agent_id][j]]

                    # remove agent's own id from list of neighbors
                    if agent_id in neighbor_conn_ids:
                        neighbor_conn_ids.remove(agent_id)

            # combine with direct connection ids and sort
            ids = list(set(sorted(ids + neighbor_conn_ids)))

            # divide out direct measurement connections and all connections
            connections_new = list(set(sorted(neighbor_conn_ids + self.connections[agent_id])))
            meas_connections = self.connections[agent_id]
            self.meas_connections = meas_connections
            self.agent_connections = connections_new

            est_state_length = len(ids)

            # find the connections, and therefore intersecting states of all connections, used for similarity transforms in CI updates
            self.neighbor_connections = {}
            self.neighbor_meas_connections = {}
            # loop through all connections of each of self's connections
            for i in self.connections[agent_id]:
                if i is not agent_id:
                    neighbor_agent_id = deepcopy(i)
                    ids_new = sorted(deepcopy(self.connections[neighbor_agent_id]))
                    ids_new.append(neighbor_agent_id)

                    # build list of distance one and distance two neighbors for each agent
                    # each agent gets full list of connections
                    neighbor_conn_ids = []
                    for j in range(0,len(self.connections[neighbor_agent_id])):
                        for k in range(0,len(self.connections[self.connections[neighbor_agent_id][j]])):
                            if not self.connections[self.connections[neighbor_agent_id][j]][k] in neighbor_conn_ids:
                                neighbor_conn_ids += deepcopy(self.connections[self.connections[neighbor_agent_id][j]])

                            # remove agent's own id from list of neighbors
                            if neighbor_agent_id in neighbor_conn_ids:
                                neighbor_conn_ids.remove(neighbor_agent_id)

                    # combine with direct connection ids and sort
                    ids_new = list(set(sorted(ids_new + neighbor_conn_ids)))

                    # divide out direct measurement connections and all connections
                    neighbor_connections_new = list(set(sorted(neighbor_conn_ids + deepcopy(self.connections[neighbor_agent_id]))))
                    meas_connections_new = deepcopy(self.connections[neighbor_agent_id])
                    self.neighbor_meas_connections[i] = deepcopy(meas_connections_new)
                    self.neighbor_connections[i] = deepcopy(neighbor_connections_new)

            # construct local estimate
            # TODO: remove hardcoded 6
            n = (est_state_length)*6
            F,G,Q = globals()[dynamics_fxn](self.dt,est_state_length,**dynamics_fxn_params)

            # sort all connections
            # ids = sorted([agent_id] + connections_new)
            # create initial state estimate by grabbing relevant ground truth from full ground truth vector
            x0 = np.array([])
            for j in range(0,len(ids)):
                if j == agent_id:
                    x = x_true_vec[ids[j]*6:ids[j]*6+6]
                else:
                    x = np.array((0,0,0,0,0,0)).transpose()
                x0 = np.hstack( (x0,x) )

            P0 = 5000*np.eye(6*est_state_length)

            local_filter = ETKF(F,G,0,0,Q,np.array(R_abs),np.array(R_rel),
                                x0.reshape((F.shape[0],1)),P0,self.delta,
                                agent_id,connections_new,-1)

            # construct common information estimates
            common_estimates = []
            for j in range(0,len(meas_connections)):
                
                # find unique states between direct connections
                # inter_states = set(meas_connections).intersection(self.connections[self.connections[i][j]])
                unique_states = set(meas_connections+self.connections[self.connections[agent_id][j]])
                comm_ids = list(unique_states)
                x0_comm = np.array([])
                for k in range(0,len(comm_ids)):
                    if k == agent_id:
                        x = x_true_vec[comm_ids[k]*6:comm_ids[k]*6+6]
                    else:
                        x = np.array((0,0,0,0,0,0)).transpose()
                    x0_comm = np.hstack( (x0_comm,x) )
                
                # create comm info filter initial covariance
                P0_comm = 5000*np.eye(6*len(comm_ids))

                # generate dynamics
                F_comm, G_comm, Q_comm = globals()[dynamics_fxn](self.dt,len(comm_ids),**dynamics_fxn_params)

                # remove agent id from comm ids
                if agent_id in comm_ids:
                    comm_ids.remove(agent_id)

                # create common information filter
                comm_filter = ETKF(F_comm,G_comm,0,0,Q_comm,np.array(R_abs),np.array(R_rel),
                                    x0_comm.reshape((F_comm.shape[0],1)),P0_comm,self.delta,
                                    agent_id,comm_ids,meas_connections[j])

                common_estimates.append(comm_filter)

            # create agent instance
            new_agent = Agent(agent_id,connections_new,meas_connections,neighbor_conn_ids,
                                local_filter,common_estimates,x_true_vec[6*i:6*i+6],
                                0,len(x0)*self.tau_state_goal,len(x0)*self.tau,
                                self.use_adaptive_tau)

            agents.append(new_agent)

        return agents

    def update(self,dynamics_fxn='lin_ncv',sensors={}):
        """
        Update simulation by one timestep dt. First updates all ground truth.
        Then updates baseline and all agents filters.
        """

        # print estimates for verification
        # for i,a in enumerate(self.agents):
        #     print('agent {} est: {}'.format(i+1,a.local_filter.x))

        # input('press enter to continue to next time step')

        # get dynamics abnd sensor noise
        F_local, G_local, _ = globals()[dynamics_fxn](self.dt,1)
        R_abs = sensors['lin_abs_pos']['noise']
        R_rel = sensors['lin_rel_range']['noise']

        # initialize msg inbox and ci inbox
        inbox = []
        ci_inbox = []
        for i in range(0,self.num_agents):
            inbox.append([])
            ci_inbox.append([])

        # generate control input
        agent_control_input = np.array( ((2*np.cos(0.75*self.sim_time)),(2*np.sin(0.75*self.sim_time)),(0)) )
        all_control_input = np.tile(agent_control_input,self.num_agents)

        # print('agent crtl input: {}'.format(agent_control_input))

        # propagate baseline estimate
        self.baseline_filter.predict(all_control_input)

        # agent updates -- ground truth, local thresholding, and processing
        for j in range(0,self.num_agents):
            msgs = []

            # propagate agent true states
            # w = self.w_data[0,j][:,self.sim_time_step] # loaded from data file
            # print('w: {}'.format(w))
            w = np.random.multivariate_normal([0,0,0,0,0,0],Q_local_true).transpose()
            self.agents[j].true_state.append(np.dot(F_local,self.agents[j].true_state[-1]) 
                + np.dot(G_local,agent_control_input) + w)

            # generate measurements
            if self.agents[j].agent_id in self.sensors['lin_abs_pos']['agents']:
                # simulate measurement noise
                # v = self.v_data[0,j][:,self.sim_time_step] # loaded from data file
                # print('v_abs: {}'.format(v))
                v = np.random.multivariate_normal([0,0,0],R_abs)
                # create measurement with agent true state and simmed noise
                y_abs = np.dot(H_local,self.agents[j].true_state[-1]) + v
                # generate message structure -> note dest will change in thresholding
                y_abs_msg = MeasurementMsg(self.agents[j].agent_id,
                                            self.agents[j].agent_id,
                                            self.agents[j].agent_id,
                                            [1,1,1],'abs',y_abs)
                # add msg to queue
                msgs.append(y_abs_msg)

                # comm drop simulation
                if np.random.binomial(1,1-self.msg_drop_prob):
                    self.baseline_filter.update(y_abs,'abs',self.agents[j].agent_id,
                                                self.agents[j].agent_id)

            for k in range(0,len(self.agents[j].meas_connections)):
                # simulate measurement noise
                # v_rel = self.v_rel_data[0,j][:,self.sim_time_step]
                # print('v_rel: {}'.format(v_rel))
                v_rel = np.random.multivariate_normal([0,0,0],R_rel)
                # create measurement with agent and target true states and simmed noise
                y_rel = np.dot(H_rel,np.hstack( (self.agents[j].true_state[-1],
                            self.agents[self.agents[j].meas_connections[k]].true_state[-1]) )) + v_rel
                # generate message structure -> note dest will change in thresholding
                y_rel_msg = MeasurementMsg(self.agents[j].agent_id,
                                            self.agents[j].meas_connections[k],
                                            self.agents[j].meas_connections[k],
                                            [1,1,1],'rel',y_rel)
                # add msg to queue
                msgs.append(y_rel_msg)

                # comms drop simulation
                if np.random.binomial(1,1-self.msg_drop_prob):
                    self.baseline_filter.update(y_rel,'rel',self.agents[j].agent_id,
                                                self.agents[j].meas_connections[k])

            # locally process measurements
            # agent_control_input = control_input[2*j:2*j+1,self.sim_time_step]
            outgoing = self.agents[j].process_local_measurements(agent_control_input,msgs)

            # add outgoing measurements to agent inboxes
            for k in outgoing:
                dest = k.dest
                inbox[dest].append(k)
                self.all_msgs.append(k)

        # agent update -- process received messages
        for j in range(0,self.num_agents):
            self.agents[j].process_received_measurements(inbox[j])

        # covariance intersection
        # this loop is a proxy for a service request and response framework.
        # The agent that triggers CI creates a request in the form of a StateMsg message
        # populated with its info, and sends it to its direct (distance-one) connections.
        # The connections popluate the remaining empty fields of the received message with 
        # their own information, and responsd with a new message of the same type with their info.
        # Everyone adds these messages to their inbox to be processed.
        for j, agent in enumerate(self.agents):
            
        #     # check covariance trace for triggering CI
            if np.trace(agent.local_filter.P) > agent.tau:
                agent.ci_trigger_cnt += 1
                agent.ci_trigger_rate = agent.ci_trigger_cnt / (i-1)

                for conn_id in agent.meas_connections:
                    
                    # generate message
                    msg_a = agent.gen_ci_message(conn_id,list(self.agents[conn_id].connections))
                    msg_b = self.agents[conn_id].gen_ci_message(agent.agent_id,list(agent.connections))

                    # compress state messages
                    if self.quantization and self.diagonalization:

                        # create element types list: assumes position, velocity alternating structure
                        element_types = []
                        for el_idx in range(0,msg_a.est_cov.shape[0]):
                            if el_idx % 2 == 0: element_types.append('position')
                            else: element_types.append('velocity')

                        # first diagonalize
                        cova_diag = covar_diagonalize(msg_a.est_cov)
                        covb_diag = covar_diagonalize(msg_b.est_cov)

                        # then quantize
                        print(cova_diag.shape)
                        print(msg_a.state_est.shape)
                        bits_a = self.quantizer.state2quant(msg_a.state_est, cova_diag, element_types, diag_only=True)
                        bits_b = self.quantizer.state2quant(msg_b.state_est, covb_diag, element_types, diag_only=True)

                        # then decompress
                        meana_quant, cova_quant = self.quantizer.quant2state(bits_a[0], 2*cova_diag.shape[0], element_types, diag_only=True)
                        meanb_quant, covb_quant = self.quantizer.quant2state(bits_b[0], 2*covb_diag.shape[0], element_types, diag_only=True)

                        print(cova_quant.shape)
                        print(msg_a.est_cov.shape)
                        print(covb_quant.shape)
                        print(msg_b.est_cov.shape)
                        assert(cova_quant.shape == msg_a.est_cov.shape)

                        # add back to state messages
                        msg_a.state_est = meana_quant
                        msg_a.est_cov = cova_quant
                        msg_b.state_est = meanb_quant
                        msg_b.est_cov = covb_quant

                    elif self.quantization:

                        # create element types list: assumes position, velocity alternating structure
                        element_types = []
                        for el_idx in range(0,msg_a.est_cov.shape[0]):
                            if el_idx % 2 == 0: element_types.append('position')
                            else: element_types.append('velocity')

                        # quantize
                        bits_a = self.quantizer.state2quant(msg_a.state_est, msg_a.est_cov, element_types)
                        bits_b = self.quantizer.state2quant(msg_b.state_est, msg_b.est_cov, element_types)

                        # then decompress
                        meana_quant, cova_quant = self.quantizer.quant2state(bits_a[0], int(msg_a.est_cov.shape[0] + (msg_a.est_cov.shape[0]**2 + msg_a.est_cov.shape[0])/2), element_types)
                        meanb_quant, covb_quant = self.quantizer.quant2state(bits_b[0], int(msg_b.est_cov.shape[0] + (msg_b.est_cov.shape[0]**2 + msg_b.est_cov.shape[0])/2), element_types)

                        # add back to state messages
                        meana_quant = np.reshape(meana_quant,msg_a.state_est.shape)
                        msg_a.state_est = meana_quant
                        msg_a.est_cov = cova_quant
                        meanb_quant = np.reshape(meanb_quant,msg_b.state_est.shape)
                        msg_b.state_est = meanb_quant
                        msg_b.est_cov = covb_quant

                    elif self.diagonalization:
                        # diagonalize
                        cova_diag = covar_diagonalize(msg_a.est_cov)
                        covb_diag = covar_diagonalize(msg_b.est_cov)

                        msg_a.est_cov = cova_diag
                        msg_b.est_cov = covb_diag

                    # add messages to ci inbox
                    ci_inbox[conn_id].append(deepcopy(msg_a))
                    ci_inbox[agent.agent_id].append(msg_b)

        # # process inbox messages
        for j, msg_list in enumerate(ci_inbox):
            self.agents[j].process_ci_messages(msg_list)

        # update state history and mse 
        for j, agent in enumerate(self.agents):
            agent.local_filter.state_history.append(agent.local_filter.x)
            agent.local_filter.cov_history.append(agent.local_filter.P)

            for filter_ in agent.common_estimates:
                filter_.state_history.append(filter_.x)
                filter_.cov_history.append(filter_.P)

            # record agent (position) MSE and baseline MSE
            _,idx = agent.get_location(agent.agent_id)
            agent_mse = np.linalg.norm(np.take(agent.local_filter.x,[idx[0],idx[2],idx[4]]) - np.take(agent.true_state[-1],[0,2,4]),ord=2)**2 
            agent.mse_history.append(agent_mse)

        # update baseline est and cov histories
        self.baseline_filter.state_history.append(self.baseline_filter.x)
        self.baseline_filter.cov_history.append(self.baseline_filter.P)

        # input('press enter to continue to next time step')

    def run_sim(self,print_strs=None):
        """
        Run simulation with sim instance parameters, and package results data.
        Called externally after initialization of sim instance.

        Inputs:

            print_strs -- list of status strings to be printed to terminal, e.g. sim params

        Returns:

            sim_data -- packaged simulation data to be processed
        """
        print_strs.append([])

        # sim update loop
        while self.sim_time < self.max_time+self.dt:
            # update printed status messages
            sim_time_str = 'Sim time: {} of {} sec, {}% complete'.format(
                self.sim_time,self.max_time,100*(self.sim_time/self.max_time))
            print_strs[3] = sim_time_str
            self.print_status(print_strs)

            # update agents
            self.update(self.dynamics,self.sensors)

            # increament simulation time
            self.sim_time += self.dt
            self.sim_time_step += 1

        # create empty numpy arrays to store results -> rows=time, columns=agents
        mse_results_array = np.empty((self.sim_time_step,self.num_agents))
        local_filter_history = np.empty(())

        baseline_state_history = np.array(self.baseline_filter.state_history)
        baseline_cov_history = np.array(self.baseline_filter.cov_history)

        agent_msgs_total = []
        agent_msgs_sent = []
        agent_ci_total = []
        agent_ci_rate = []

        agent_state_histories = []
        agent_cov_histories = []
        agent_true_states = []

        agent_state_error = []
        baseline_state_error = []

        for i,a in enumerate(self.agents):
            mse_results_array[:,i] = np.array(a.mse_history)

            agent_state_histories.append(np.array(a.local_filter.state_history))
            agent_cov_histories.append(np.array(a.local_filter.cov_history))
            agent_true_states.append(np.array(a.true_state))

            agent_msgs_total.append(a.total_msgs)
            agent_msgs_sent.append(a.msgs_sent)
            agent_ci_total.append(a.ci_trigger_cnt)
            agent_ci_rate.append(a.ci_trigger_rate)

            # loc,idx = a.get_location(a.agent_id)
            # agent_state_error.append(np.array(a.local_filter.state_history[idx])-np.array(a.true_state))
            # baseline_state_error.append(np.array(baseline_state_history[6*a.agent_id:6*a.agent_id+6])-np.array(a.true_state))
            
        # print ci_process_worst_case_time for each agent
        # for i,a in enumerate(self.agents):
            # print('Agent {} worst case time: {}'.format(i,a.ci_process_worst_case_time))

        # package simulation results
        # res = package_results()
        # results_dict = {'baseline': self.baseline_filter, 'agents': self.agents}
        results_dict = {'agent_mse': mse_results_array,
                        'agent_state_histories': agent_state_histories,
                        'agent_cov_histories': agent_cov_histories,
                        'agent_true_states': agent_true_states,
                        # 'agent_state_error': agent_state_error,
                        'baseline_state_history': baseline_state_history,
                        'baseline_cov_history': baseline_cov_history,
                        # 'baseline_state_error': baseline_state_error,
                        'agent_msgs_total': agent_msgs_total,
                        'agent_msgs_sent': agent_msgs_sent,
                        'agent_ci_total': agent_ci_total,
                        'agent_ci_rate': agent_ci_rate}
                        
        # create metadata dictionary
        metadata_dict = {'max_time': self.max_time, 
                        'dt': self.dt,
                        'connections': self.connections,
                        'num_agents': self.num_agents,
                        'delta': self.delta,
                        'tau_goal': self.tau_state_goal,
                        'msg_drop_prob': self.msg_drop_prob,
                        'dynamics': self.dynamics,
                        'sensors': self.sensors}

        return {'metadata': metadata_dict, 'results': results_dict}

    def print_status(self,print_strs):
        """
        Print simulation status, formatted nicely w/ input strs.

        Inputs:

            print_strs -- list of strings to be printed to console
        """
        os.system('clear')
        print('------------------------------------------------------')
        for str_ in print_strs:
            print(str_)
        print('------------------------------------------------------')


H_local = np.array( ((1,0,0,0,0,0),
                    (0,0,1,0,0,0),
                    (0,0,0,0,1,0)) )
H_rel = np.array( ((1,0,0,0,0,0,-1,0,0,0,0,0),
                    (0,0,1,0,0,0,0,0,-1,0,0,0),
                    (0,0,0,0,1,0,0,0,0,0,-1,0)) )
Q_local_true = np.array( ((0.0003,0.005,0,0,0,0),
                            (0.005,0.1,0,0,0,0),
                            (0,0,0.0003,0.005,0,0),
                            (0,0,0.005,0.1,0,0),
                            (0,0,0,0,0.0003,0.005),
                            (0,0,0,0,0.005,0.1)) )


# main driver function

def main(plot=False,cfg_path=None,save_path=None):
    """
    Main driver function. Called when running sim.py directly or as a module.

    Inputs:

        plot [optional] -- flag to determine if sim results will be plotted
        cfg_path [optional] -- string path to config file (full path)
        save_path [optional] --  string path to save location for sim results (full path)

    Returns:

        none
    """

    # load sim config
    if cfg_path is None:
        cfg_path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                        '../config/config.yaml'))
    cfg = load_config(cfg_path)

    # specify path for saving data
    if save_path is None:
        save_path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                        '../data/'))

    # make new data directory for sim files
    save_path = make_data_directory(save_path)

    # extract config params
    num_mc_sim = cfg['num_mc_sims']
    delta_values = cfg['delta_values']
    tau_values = cfg['tau_values']
    msg_drop_prob_values = cfg['msg_drop_prob_values']

    # initilize results container
    results = []

    # sim counter
    sim_cnt = 1
    total_sims = len(delta_values)*len(tau_values)*len(msg_drop_prob_values)*num_mc_sim

    for i in delta_values:
        for j in tau_values:
            for k in msg_drop_prob_values:

                # numpy array for monte carlo averaged results
                mc_mse_results = np.empty((int(cfg['max_time']/cfg['dt'] + 1),len(cfg['agent_cfg']['conns']),num_mc_sim))
                # mc_time_trace_results = np.empty((int(cfg['max_time']/cfg['dt'] + 1),len(cfg['agent_cfg']['conns']),num_mc_sim))
                # mc_baseline_results = np.empty((int(cfg['max_time']/cfg['dt'] + 1),len(cfg['agent_cfg']['conns']),num_mc_sim))

                mc_msgs_total = np.empty((len(cfg['agent_cfg']['conns']),num_mc_sim))
                mc_msgs_sent = np.empty((len(cfg['agent_cfg']['conns']),num_mc_sim))
                mc_ci_total = np.empty((len(cfg['agent_cfg']['conns']),num_mc_sim))
                mc_ci_rate = np.empty((len(cfg['agent_cfg']['conns']),num_mc_sim))

                for m in range(1,num_mc_sim+1):
                    # create simulation status strings to be printed
                    sim_print_str = 'Initializing simulation {} of {}'.format(sim_cnt,total_sims)
                    param_print_str = 'Params: delta={},\t tau={}, \t msg drop prob={}'.format(i,j,k)
                    mc_print_str = 'Monte Carlo sim {} of {}'.format(m,num_mc_sim)

                    # create sim instance w/ sim params
                    sim = SimInstance(delta=i,tau=j,msg_drop_prob=k,
                                        baseline_cfg=cfg['baseline_cfg'],
                                        agent_cfg=cfg['agent_cfg'],
                                        max_time=cfg['max_time'],
                                        dt=cfg['dt'],
                                        use_adaptive_tau=cfg['use_adaptive_tau'],
                                        fixed_rng=cfg['fixed_rng'],
                                        process_noise=False,
                                        sensor_noise=False,
                                        quantization_flag=cfg['quantization'],
                                        diagonalization_flag=cfg['diagonalization'])
                    # run simulation
                    res = sim.run_sim([sim_print_str,param_print_str,mc_print_str])
                    # add results to results container
                    # results.append(res)
                    mc_mse_results[:,:,m-1] = res['results']['agent_mse']

                    mc_msgs_total[:,m-1] = res['results']['agent_msgs_total']
                    mc_msgs_sent[:,m-1] = res['results']['agent_msgs_sent']
                    mc_ci_total[:,m-1] = res['results']['agent_ci_total']
                    mc_ci_rate[:,m-1] = res['results']['agent_ci_rate']

                    sim_cnt += 1

                mc_avg_mse_results = np.mean(mc_mse_results,axis=2)

                mc_avg_msgs_total = np.mean(mc_msgs_total,axis=1)
                mc_avg_msgs_sent = np.mean(mc_msgs_sent,axis=1)
                mc_avg_ci_total = np.mean(mc_ci_total,axis=1)
                mc_avg_ci_rate = np.mean(mc_ci_rate,axis=1)

                results = {'mse': mc_avg_mse_results,
                            'msgs_total': mc_avg_msgs_total,
                            'msgs_sent': mc_avg_msgs_sent,
                            'ci_total': mc_avg_ci_total,
                            'ci_rate': mc_avg_ci_rate}

                # create metadata dictionary
                metadata_dict = {'num_mc_sim': num_mc_sim,
                                'delta_value': (i,delta_values),
                                'tau_value': (j,tau_values),
                                'msg_drop_prob_value': (k,msg_drop_prob_values)}

                # create filename
                file_name = 'delta{}_tau{}_drop{}_mc{}'.format(i,j,k,num_mc_sim).replace('.','')

                # save data to pickle file
                save_sim_data(metadata_dict,results,save_path,file_name)

    # write metadata file for data
    sim_metadata = {'cfg': cfg}
    write_metadata_file(save_path,sim_metadata)


    # if plot flag is set, plot results
    if plot:
        time_trace_plots(results,[0])

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Run a set of simulations using the ET-DDF framework.')
    parser.add_argument('-p','--plot',dest='plot_flag',action='store_true',
                            help='plot the simulation results at the end (not suggested unless you are only running one sim')
    parser.add_argument('-c','--config-path', dest='config_path',action='store',
                            help='specify the (full) path to a sim config file.')
    parser.add_argument('-s','--save-path', dest='save_path',action='store',
                            help='specify the (full) path to the location where sim data will be saved.')
    args = parser.parse_args()

    # run the sim driver with command line args
    main(plot=args.plot_flag,cfg_path=args.config_path,save_path=args.save_path)
