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

    # Q_local_true = [0.0003 0.005 0 0;
    #                 0.005 0.1 0 0;
    #                 0 0 0.0003 0.005;
    #                 0 0 0.005 0.1]
                
    q = np.array( ((0.0017,0.025,0,0),
                (0.025,0.5,0,0),
                (0,0,0.0017,0.025),
                (0,0,0.025,0.5)) )

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
    Q = q

    for i in range(1,n):
        F = block_diag(F,f)
        G = block_diag(G,g)
        Q = block_diag(Q,q)

    return F, G, Q

def lin_ncv_3d(dt,n=1,coplanar=False):
    """ Nearly-constant velocity linear dynamics, but in 3d!

    Params:
        dt -- timestep [seconds]
        n -- number of agents
        coplanar -- (optional) boolean flag for if agents are assumed to be
                    moving at a constant altitude

    Returns:
        F -- state transition matrix
        G -- control input to state matrix
    """

    # Q_local_true = [0.0003 0.005 0 0;
    #                 0.005 0.1 0 0;
    #                 0 0 0.0003 0.005;
    #                 0 0 0.005 0.1]

    if coplanar:
    
        q = np.array( ((0.0017,0.025,0,0,0,0),
                    (0.025,0.5,0,0,0,0),
                    (0,0,0.0017,0.025,0,0),
                    (0,0,0.025,0.5,0,0),
                    (0,0,0,0,0.0017,0.025),
                    (0,0,0,0,0.025,0.5)) )

        f = np.array(((1,dt,0,0,0,0),
                    (0,1,0,0,0,0),
                    (0,0,1,dt,0,0),
                    (0,0,0,1,0,0),
                    (0,0,0,0,1,dt),
                    (0,0,0,0,0,1)) )

        g = np.array(((0.5*(dt**2),0),
                    (dt,0),
                    (0,0.5*(dt**2)),
                    (0,dt),
                    (0,0),
                    (0,0)) )

    else:

        q = np.array( ((0.0017,0.025,0,0,0,0),
                    (0.025,0.5,0,0,0,0),
                    (0,0,0.0017,0.025,0,0),
                    (0,0,0.025,0.5,0,0),
                    (0,0,0,0,0.0017,0.025),
                    (0,0,0,0,0.025,0.5)) )

        f = np.array(((1,dt,0,0,0,0),
                    (0,1,0,0,0,0),
                    (0,0,1,dt,0,0),
                    (0,0,0,1,0,0),
                    (0,0,0,0,1,dt),
                    (0,0,0,0,0,1)) )

        g = np.array(((0.5*(dt**2),0,0),
                    (dt,0,0),
                    (0,0.5*(dt**2),0),
                    (0,dt,0),
                    (0,0,0.5*(dt**2)),
                    (0,0,dt)) )

    F = f
    G = g
    Q = q

    for i in range(1,n):
        F = block_diag(F,f)
        G = block_diag(G,g)
        Q = block_diag(Q,q)

    return F, G, Q

if __name__ == "__main__":

    dt = 0.1
    n = 1

    F, G = lin_ncv(dt,n)

    print(F)
    print(G) 