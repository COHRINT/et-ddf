#!/usr/bin/env python

"""
Various helper functions for packaging and loading simulation data.
"""

import os
import sys
import numpy as np

def gen_sim_data_struct(baseline,agents):
    """
    Generates sim results data struct from baseline filter object and Agent objects,
    to be used in plotting and analysis tools.

    Keywork arguments:

        baseline -- baseline filter, w/ truth data and state history
        agents -- list of Agent objects, each w/ filters

    Returns:

        struct -- results data structure
    """
    pass