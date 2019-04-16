#!/usr/bin/env python

"""
Various helper functions for packaging and loading simulation data,
and handling messages during simulation.
"""

import os
import sys
import pickle
import time
import numpy as np

def gen_sim_data_struct(baseline,agents):
    """
    Generates sim results data struct from baseline filter object and Agent objects,
    to be used in plotting and analysis tools.

    Keyword arguments:

        baseline -- baseline filter, w/ truth data and state history
        agents -- list of Agent objects, each w/ filters

    Returns:

        struct -- results data structure
    """
    pass

def package_results(baseline,agents,all_msgs):
    pass


def save_sim_data(metadata, results, save_path):
    """
    Save data from simulation to pickle file specified by file path.
    
    Inputs:

        metadata -- dictionary of metadata values for sim suite
        results -- array of results structures from all sims
        save_path -- string file path for file to be saved

    Returns:

        none
    """

    # construct filename
    fn = os.path.join(save_path,'sim_' + time.strftime("%Y%m%d-%H%M%S") + '.pckl')

    print('Saving sim results to {}'.format(fn))
    save_obj = {'metadata': metadata, 'results_obj': results}

    try:
        with open(fn,'wb') as f:
            pickle.dump(save_obj,f)
    except IOError as e:
        print('Problem saving file: {}'.format(e))

