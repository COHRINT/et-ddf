#!/usr/bin/env python

"""
Simulation instance class implementation.
"""

import os
import sys
import yaml
import numpy as np
import scipy.linalg
# import pudb; pudb.set_trace()
from copy import deepcopy

from .agent import Agent
from .filters.kf import KF
from .filters.etkf import ETKF
from .dynamics import *
from .helpers.config_handling import load_config
from .helpers.msg_handling import MeasurementMsg, StateMsg
from .helpers.data_handling import package_results

class SimInstance(object):
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

    def __init__(self,delta,tau,msg_drop_prob,baseline_cfg,
                    agent_cfg,max_time=20,dt=0.1,use_adaptive_tau=True,
                    fixed_rng=None,process_noise=False,sensor_noise=False):

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

        self.fixed_rng = fixed_rng
        if self.fixed_rng is not None:
            np.random.seed(self.fixed_rng)
            print('Fixing random number generator with seed: {}'.format(self.fixed_rng))

        self.all_msgs = []

        # generate ground truth starting positions
        x_true_vec = self.init_ground_truth()

        # create baseline filter and agetns from config
        self.baseline_filter = self.init_baseline(x_true_vec,
                                                    baseline_cfg['dynamics_fxn'],
                                                    baseline_cfg['sensors'])
        self.agents = self.init_agents(x_true_vec,
                                        agent_cfg['dynamics_fxn'],
                                        agent_cfg['sensors'])
        
        print('Initialized baseline and agents')

    def init_ground_truth(self):
        """
        Create ground truth initial position for each agent
        """
        x_true_vec = np.zeros(4*self.num_agents)
        for i in range(0,self.num_agents):
            x_true = np.array((i*10,0,i*10,0)).T + np.random.normal([0,0,0,0],[5,0.5,5,0.5])
            x_true_vec[4*i:4*i+4] = x_true

        return x_true_vec

    def init_baseline(self,x_true_vec,dynamics_fxn='lin_ncv',sensors={}):
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
        P0_full = 100*np.eye(4*self.num_agents)

        # create filter instance
        baseline_filter = KF(F_full,G_full,0,0,Q_full,R_abs,R_rel,x0_full,P0_full,0)

        return baseline_filter

    def init_agents(self,x_true_vec,dynamics_fxn='lin_ncv',sensors={}):
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

            agent_id = i

            ids = deepcopy(self.connections[i])
            ids.append(agent_id)
            ids.sort()

            # build list of distance one and distance two neighbors for each agent
            neighbor_conn_ids = []
            for j in range(0,len(self.connections[i])):
                for k in range(0,len(self.connections[self.connections[i][j]])):
                    # if not any(self.connections[self.connections[i][j]][k] == x for x in neighbor_conn_ids):
                    #     neighbor_conn_ids += self.connections[self.connections[i][j]]
                    if not self.connections[self.connections[i][j]][k] in neighbor_conn_ids:
                        neighbor_conn_ids += self.connections[self.connections[i][j]]

                    # remove agent's own id from list of neighbors
                    if agent_id in neighbor_conn_ids:
                        neighbor_conn_ids.remove(agent_id)

            # combine with direct connection ids and sort
            ids = list(set(sorted(ids + neighbor_conn_ids)))

            # divide out direct measurement connections and all connections
            connections_new = list(set(sorted(neighbor_conn_ids + self.connections[i])))
            meas_connections = self.connections[i]

            est_state_length = len(ids)

            # construct local estimate
            n = (est_state_length)*4
            F,G,Q = globals()[dynamics_fxn](self.dt,est_state_length)

            # sort all connections
            ids = sorted([agent_id] + connections_new)
            # create initial state estimate by grabbing relevant ground truth from full ground truth vector
            x0 = np.array([])
            for j in range(0,len(ids)):
                x0 = np.hstack( (x0,x_true_vec[ids[j]*4:ids[j]*4+4]) )

            P0 = 100*np.eye(4*est_state_length)

            local_filter = ETKF(F,G,0,0,Q,np.array(R_abs),np.array(R_rel),
                                x0.reshape((F.shape[0],1)),P0,self.delta,
                                agent_id,connections_new,-1)

            # construct common information estimates
            common_estimates = []
            for j in range(0,len(meas_connections)):
                
                # find unique states between direct connections
                # inter_states = set(meas_connections).intersection(self.connections[self.connections[i][j]])
                unique_states = set(meas_connections+self.connections[self.connections[i][j]])
                comm_ids = list(unique_states)
                x0_comm = np.array([])
                for k in range(0,len(comm_ids)):
                    x0_comm = np.hstack( (x0_comm,x_true_vec[comm_ids[k]*4:comm_ids[k]*4+4]) )
                
                # create comm info filter initial covariance
                P0_comm = 100*np.eye(4*len(comm_ids))

                # generate dynamics
                F_comm, G_comm, Q_comm = globals()[dynamics_fxn](self.dt,len(comm_ids))

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
                                local_filter,common_estimates,x_true_vec[4*i:4*i+4],
                                self.msg_drop_prob,len(x0)*self.tau_state_goal,len(x0)*self.tau)

            agents.append(new_agent)


        return agents

    def update(self,dynamics_fxn='lin_ncv',sensors={}):
        """
        Update simulation by one timestep dt. First updates all ground truth.
        Then updates baseline and all agents filters.
        """
        # get dynamics abnd sensor noise
        F_local, G_local, _ = globals()[dynamics_fxn](self.dt,1)
        R_abs = sensors['lin_abs_pos']['noise']
        R_rel = sensors['lin_rel_range']['noise']

        # initialize msg inbox
        inbox = []
        for i in range(0,self.num_agents):
            inbox.append([])

        # agent updates -- ground truth, local thresholding, and processing
        for j in range(0,self.num_agents):
            msgs = []

            # generate control input
            agent_control_input = np.array( ((2*np.cos(0.75*self.sim_time)),(2*np.sin(0.75*self.sim_time))) )

            # propagate agent true states
            w = np.random.multivariate_normal([0,0,0,0],Q_local_true).transpose()
            self.agents[j].true_state.append(np.dot(F_local,self.agents[j].true_state[-1]) 
                + np.dot(G_local,agent_control_input) + w)

            # generate measurements
            if self.agents[j].agent_id in self.sensors['lin_abs_pos']['agents']:
                # simulate measurement noise
                v = np.random.multivariate_normal([0,0],R_abs)
                # create measurement with agent true state and simmed noise
                y_abs = np.dot(H_local,self.agents[j].true_state[:,-1]) + v
                # generate message structure -> note dest will change in thresholding
                y_abs_msg = MeasurementMsg(self.agents[j].agent_id,
                                            self.agents[j].agent_id,
                                            self.agents[j].agent_id,
                                            [1,1],'abs',y_abs)
                # add msg to queue
                msgs.append(y_abs_msg)

                # comm drop simulation
                if np.random.binomial(1,1-self.msg_drop_prob):
                    self.baseline_filter.update(y_abs,'abs',self.agents[j].agent_id,
                                                self.agents[j].agent_id)

            for k in range(0,len(self.agents[j].meas_connections)):
                # simulate measurement noise
                v_rel = np.random.multivariate_normal([0,0],R_rel)
                # create measurement with agent and target true states and simmed noise
                y_rel = np.dot(H_rel,np.hstack( (self.agents[j].true_state[-1],
                            self.agents[self.agents[j].meas_connections[k]].true_state[-1]) )) + v_rel
                # generate message structure -> note dest will change in thresholding
                y_rel_msg = MeasurementMsg(self.agents[j].agent_id,
                                            self.agents[j].meas_connections[k],
                                            self.agents[j].meas_connections[k],
                                            [1,1],'rel',y_rel)
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
        # TODO: THIS ENTIRE GODDAMN THING

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
        while self.sim_time < self.max_time:
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
            
        # package simulation results
        # res = package_results()

        return None

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


H_local = np.array( ((1,0,0,0),
                    (0,0,1,0)) )
H_rel = np.array( ((1,0,0,0,-1,0,0,0),
                    (0,0,1,0,0,0,-1,0)) )
Q_local_true = np.array( ((0.0003,0.005,0,0),
                            (0.005,0.1,0,0),
                            (0,0,0.0003,0.005),
                            (0,0,0.005,0.1)) )


# main driver function

def main():
    """
    Main driver function. Called when running sim.py directly.
    """

    # load sim config
    load_path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                    '../config/config.yaml'))
    cfg = load_config(load_path)

    # specify path for saving data
    save_path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                    '../data/'))

    # run simulation driver
    # status = main(**cfg)

    # extract config params

    # number of Monte-Carlo sims
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
                for m in range(0,num_mc_sim):
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
                                        sensor_noise=False)
                    # run simulation
                    res = sim.run_sim([sim_print_str,param_print_str,mc_print_str])
                    # add results to results container
                    results.append(res)

if __name__ == "__main__":
    main()
