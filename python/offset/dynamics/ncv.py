#!/usr/bin/env python

"""
Nearly constant velocity linear dynamics. Tracks E and N position and velocity.
"""

import numpy as np
from scipy.linalg import block_diag

def lin_ncv(dt,n=1):
    """ Nearly-constant velocity linear dynamics.

    Params:
        dt -- timestep [seconds]
        n -- number of agents

    Returns:
        F -- state transition matrix
        G -- control input to state matrix
    """

    f = np.array(((1,dt,0,0),
                (0,1,0,0),
                (0,0,1,dt),
                (0,0,0,1)) )

    g = np.array(((0.5*(dt**2),0),
                (dt,0),
                (0,0.5*(dt**2)),
                (0,dt)) )

    F = f
    G = g

    for i in range(1,n):
        F = block_diag(F,f)
        G = block_diag(G,g)

    return F, G

if __name__ == "__main__":

    dt = 0.1
    n = 1

    F, G = lin_ncv(dt,n)

    print(F)
    print(G) 