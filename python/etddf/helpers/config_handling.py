#!/usr/bin/env python

import os
import sys
import yaml

"""
Functions to handle loading and saving simulation configs.
"""

def load_config(fn):
    """Loads yaml simulation config."""
    with open(fn,'r') as f:
        cfg = yaml.safe_load(f)
    return cfg

if __name__ == "__main__":
    pass