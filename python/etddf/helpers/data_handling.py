#!/usr/bin/env python

"""
Various helper functions for packaging and loading simulation data,
and handling messages during simulation.
"""

import os
import sys
import pickle
import time
import yaml
import numpy as np

from etddf.agent import Agent
from etddf.filters.etkf import ETKF
from etddf.filters.kf import KF

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


def save_sim_data(metadata, results, save_path, file_name):
    """
    Save data from simulation to pickle file specified by file path.
    
    Inputs:

        metadata -- dictionary of metadata values for sim suite
        results -- array of results structures from all sims
        save_path -- string file path for file to be saved
        file_name -- string of filename

    Returns:
 
        none
    """

    # construct filename
    fn = os.path.join(save_path,file_name + '_' + time.strftime("%Y%m%d-%H%M%S") + '.pckl')

    print('Saving sim results to {}'.format(fn))
    save_obj = {'metadata': metadata, 'results': results}

    try:
        with open(fn,'wb') as f:
            pickle.dump(save_obj,f)
    except IOError as e:
        print('Problem saving file: {}'.format(e))

def make_data_directory(dir_path,dir_name=None):
    """
    Create new directory for data. Returns full path to new directory.
    """
    if dir_name is None:
        dir_name = 'et-ddf-data_'

    path = os.path.abspath(os.path.join(dir_path,'et-ddf-data_'+time.strftime("%Y%m%d-%H%M%S")))

    if not os.path.exists(path):
        os.makedirs(path)
    else:
        print('Data directory already exists!')

    return path

def write_metadata_file(path,metadata):
    """
    Create simulation metadata file using passed metadata dictionary.
    """
    with open(os.path.join(path,'sim_metadata.yaml'),'w') as f:
        yaml.dump(metadata,f,default_flow_style=True)

def load_metadata(dir_path):
    """
    Load simulation metadata yaml file using directory path.
    """
    with open(os.path.join(dir_path,'sim_metadata.yaml'),'r') as f:
        metadata = yaml.load(f,Loader=yaml.SafeLoader)

    return metadata

def load_sim_data(save_path):
    """
    Load saved simulation data from pickle file.

    Inputs:

        save_path -- full file path to sim data pickle

    Returns:

        data -- unpickled data, includes metadata, and results
    """

    # construct filename
    fn = os.path.abspath(os.path.join(os.path.dirname(__file__),save_path))

    try:
        with open(fn,'rb') as f:
            data = pickle.load(f)
    except IOError as e:
        print('Problem loading file: {}'.format(e))
        data = None

    return data

if __name__ == "__main__":

    pass