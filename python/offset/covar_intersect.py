#!/usr/bin/env python

__author__ = "Ian Loefgren"
__date__ = "1.7.2018"

import numpy as np
import math
# import pudb; pudb.set_trace()

def covar_intersect(xa,xb,Pa,Pb,alpha=-1):
    """
    Computes a conservative, consistent fusion of two covariance matrices and
    associated means. See Julier, Uhlmann. "A Non-divergent Estimation
    Algorithm in the Presence of Unknown Correlations" 1997.

    Usage:
        xa - mean A, must be column vector
        xb - mean B, must be column vector
        Pa - covariance of A
        Pb - covariance of B

    Returns:
        xc - fused mean
        Pc - fused covariance

    """

    if alpha is -1:
        alpha = np.ones(np.shape(xa))

    # create fxn handle to find omega that minimizes tr(P)
    f = lambda omega: np.trace(np.linalg.inv(omega*np.linalg.inv(Pa) 
                + (1-omega)*np.linalg.inv(Pb))*np.diag(alpha))
    omega = gss(f,0,1)

    Pc = np.linalg.inv(omega*np.linalg.inv(Pa) + (1-omega)*np.linalg.inv(Pb))
    xc = np.dot(Pc,(omega*np.dot(np.linalg.inv(Pa),xa) + (1-omega)*np.dot(np.linalg.inv(Pb),xb)))

    return xc,Pc

def gss(f,a,b,tol=1e-5):
    """
    Golden section search method implementation. Finds zeros of 1-D functions,
    similar to bisection search.
    See https://en.wikipedia.org/wiki/Golden-section_search 

    Usage:
    f - function handle for function to minimize
    a - left bound for search
    b - right bound for search
    tol_ (optional) - tolerance for search, default 1E-5
    """

    # golden ratio
    gr = (1+math.sqrt(5))/2

    c = b-(b-a)/gr
    d = a+(b-a)/gr

    while abs(c-d) > tol:
        if f(c) < f(d):
            b = d
        else:
            a = c
        
        c = b-(b-a)/gr
        d = a+(b-a)/gr
        
    fun_min = (a+b)/2

    return fun_min

def gen_sim_transform(a_id,a_connections,b_id,b_connections,num_states=4):
    """
    Generates similarity transforms to collect states of interest in state
    estimates at the "top" of mean vectors and covaraince matrices.

    Inputs:
    
        a_id -- agent id of agent A
        a_connections -- list of connections agent A has (and appear in estimate)
        b_id -- agent id of agent B
        b_connections -- list of connections agent B has (and appear in estimate)
        num_states -- number of states per agent in estimate

    Returns:

        T -- similarity transform matrix
        il -- number of intersecting states between A and B
        inter -- intersecting agents between A and B
    """
    # create lists of all agents in each estimate
    a_conn = sorted(a_connections + [a_id])
    b_conn = sorted(b_connections + [b_id])

    # find intersection of the sets a_conn and b_conn
    inter = sorted(list(set(a_conn).intersection(set(b_conn))))
    # find indicies of intersection agents
    inter_idx = [a_conn.index(i) for i in inter]

    # find set difference between a_conn and b_conn
    diff = sorted(list(set(a_conn).difference(set(b_conn))))
    # find indicies of set difference agents
    diff_idx = [a_conn.index(i) for i in diff]

    # combine indicies
    idx = inter_idx + diff_idx

    # compute number of intersecting states
    il = num_states*len(inter)

    # construct transformation matrix
    T = np.zeros( (num_states*len(a_conn),num_states*len(a_conn)) )
    for ii in range(0,len(idx)):
        T[num_states*(idx[ii]):num_states*(idx[ii])+num_states,
            num_states*(ii):num_states*(ii)+num_states] = np.eye(num_states)

    return T, il, inter

def test_gss():
    fun1 = lambda x: x**2
    fun2 = lambda x: math.cos(x)
    fun3 = lambda x: math.exp(x)

    assert abs(gss(fun1,-5,5)) < 1e-5
    assert abs(gss(fun2,-5,5) - math.pi) < 1e-5
    assert abs(gss(fun3,-5,5) + 5) < 2e-5    

def test_covar_intersect():
    import matplotlib.pyplot as plt

    Pa = np.array([[100,0],[0,5]])
    Pb = np.array([[5,0],[0,100]])
    xa = np.array([[0],[0]])
    xb = np.array([[0],[0]])

    xc,Pc = covar_intersect(xa,xb,Pa,Pb)

    print(xc)
    print(Pc)

def test_gen_sim_transform():
    
    a_id = 3
    a_connections = [2,6,8]

    b_id = 6
    b_connections = [3,8]

    T, il, inter = gen_sim_transform(a_id,a_connections,b_id,b_connections,num_states=1)

    print('T:')
    print(T)
    print('il: {} expected: {}'.format(il,3))
    print('inter: {} expected: {}'.format(inter,[3,6,8]))


if __name__ == "__main__":
    # test_gss()
    # test_covar_intersect()
    test_gen_sim_transform()