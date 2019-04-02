#!/usr/bin/env python

"""
Simulation instance class implementation.
"""

import os
import sys
import numpy as np
import scipy.linalg

from .filters.kf import KF
from .filters.etkf import ETKF
from .dynamics.ncv import lin_ncv

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

    def __init__(self,conns,abs_meas_vec,delta,tau,msg_drop_prob,baseline_cfg,
                    agent_cfg,max_time=20,dt=0.1,use_adaptive_tau=True,
                    fixed_rng=999,process_noise=False,sensor_noise=False):

            self.max_time = max_time
            self.dt = dt
            self.sim_time = 0
            self.num_agents = len(conns)

            self.delta = delta
            self.tau = tau
            self.msg_drop_prob = msg_drop_prob
            self.use_adaptive_tau = use_adaptive_tau

            self.fixed_rng = fixed_rng

            # create baseline filter and agetns from config
            self.baseline_filter = self.init_baseline(**baseline_cfg)
            self.agents = self.init_agents(**agent_cfg)

    def init_baseline(self):
        """
        %% Create centralized KF

        Q_local_true = [0.0003 0.005 0 0;
                    0.005 0.1 0 0;
                    0 0 0.0003 0.005;
                    0 0 0.005 0.1];
                
        Q_local = [0.0017 0.025 0 0;
                    0.025 0.5 0 0;
                    0 0 0.0017 0.025;
                    0 0 0.025 0.5];

        % R_abs = 1*eye(2);
        R_abs = diag([1 1]);
        R_rel = 3*eye(2);

        % generate dynamics matrices for baseline filter
        [F_full,G_full] = ncv_dyn(dt,N);
        % Q_full = 1*eye(4*N);
        Q_full_cell = cell(1,N);
        [Q_full_cell{:}] = deal(Q_local);
        Q_full = blkdiag(Q_full_cell{:});
        x0_full = x_true_vec;
        P0_full = 100*eye(4*N);

        % create baseline filter for comparison
        baseline_filter = KF(F_full,G_full,0,0,Q_full,R_abs,R_rel,x_true_vec,P0_full,0);
        """
        F_full, G_full = lin_ncv(self.dt,self.num_agents)


    def init_agents(self,):
        """
        %% Create agents objects

        % for each platform, create dynamics models, and filters

        agents = cell(N,1);
        ci_trigger_mat = zeros(N,length(input_tvec));
        for i=1:N
            
            agent_id = i;
            
            ids = sort([agent_id,connections{i}]);
            % add shortest path to gps node to estimate
        %     gps_sp_ids = setdiff(shortest_paths{i},ids);
            neighbor_conn_ids = [];
            for j=1:length(connections{i})
                for k=1:length(connections{connections{i}(j)})
                    if ~any(neighbor_conn_ids == connections{connections{i}(j)}(k))
                        neighbor_conn_ids = [neighbor_conn_ids,connections{connections{i}(j)}];
                    end
                end
            end
            neighbor_conn_ids = neighbor_conn_ids(neighbor_conn_ids~=agent_id);
        %     ids = sort([ids,gps_sp_ids]);
            ids = sort([ids,neighbor_conn_ids]);
        %     connections_new = sort([gps_sp_ids,connections{i}]);
            connections_new = sort([neighbor_conn_ids,connections{i}]);
            meas_connections = connections{i};
            
            est_state_length = length(ids);
            
            % construct local estimates
            n = (est_state_length)*4;
            [F,G] = ncv_dyn(dt,est_state_length);
        %     Q_localfilter = 1*eye(n);
            Q_localfilter_cell = cell(1,est_state_length);
            
        %     % construct local estimates
        %     n = (length(connections{i})+1)*4;
        %     [F,G] = ncv_dyn(dt,length(connections{i})+1);
        % %     Q_localfilter = 1*eye(n);
        %     Q_localfilter_cell = cell(1,length(connections{i})+1);
            [Q_localfilter_cell{:}] = deal(Q_local);
            Q_localfilter = blkdiag(Q_localfilter_cell{:});

            ids = sort([agent_id,connections_new]);
            x0 = [];
            for j=1:length(ids)
                x0 = [x0; x_true_vec((ids(j)-1)*4+1:(ids(j)-1)*4+4,1)];
            end
            
            P0 = 100*eye(4*est_state_length);
            
            local_filter = ETKF(F,G,0,0,Q_localfilter,R_abs,R_rel,x0,P0,delta,agent_id,connections_new,-1);
            
            % construct common estimates, with intersection of states
            % loop over meas_connections, aka direct connections
            common_estimates = {};
            for j=1:length(meas_connections)
                
                % find intersection of states
        %         inter_states = intersect([agent_id,connections_new],[connections{i}(j),connections{connections{i}(j)}]); 
                inter_states = unique([meas_connections,connections{connections{i}(j)}]);
                
                % make sure common estimate state vector is ordered by agent id
        %         comm_ids = sort([agent_id,connections{i}(j)]);
                comm_ids = inter_states;
                x0_comm = [];
                
                for k=1:length(comm_ids)
                    x0_comm = [x0_comm; x_true_vec((comm_ids(k)-1)*4+1:(comm_ids(k)-1)*4+4,1)];
                end

                P0_comm = 100*eye(4*length(comm_ids));
                
                [F_comm,G_comm] = ncv_dyn(dt,length(comm_ids));
        %         Q_comm = 1*eye(8);
                Q_comm_cell = cell(1,length(comm_ids));
                [Q_comm_cell{:}] = deal(Q_local);
                Q_comm = blkdiag(Q_comm_cell{:});
                
        %         common_estimates = {};
                comm_ids = comm_ids(comm_ids~=agent_id);
            
                common_estimates{j} = ETKF(F_comm,G_comm,0,0,Q_comm,R_abs,R_rel,x0_comm,P0_comm,delta,agent_id,comm_ids,meas_connections(j));
            end
            
            agents{i} = Agent(agent_id,connections_new,meas_connections,neighbor_conn_ids,...
                                local_filter,common_estimates,x_true_vec((i-1)*4+1:(i-1)*4+4,1),...
                                msg_drop_prob,length(x0)*tau_state_goal,length(x0)*tau_state);
            
        end
        """
        pass

    def update(self):
        """
        Update simulation by one timestep dt.
        """
        pass

    def run_sim(self):
        """
        Run simulation with sim instance parameters, and package results data.
        """
        
        # sim update loop
        while self.sim_time < self.max_time:
            self.update()
            self.sim_time += self.dt

        # extract results data from agents and baseline

