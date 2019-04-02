#!/usr/bin/env python

__author__ = "Ian Loefgren"
__date__ = "1.7.2019"

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

import math

def gss(f,a,b,tol=1e-5):

    # golden ratio
    gr = (1+math.sqrt(5))/2

    c = b-(b-a)/gr;
    d = a+(b-a)/gr;

    while abs(c-d) > tol:
        if f(c) < f(d):
            b = d
        else:
            a = c
        
        c = b-(b-a)/gr
        d = a+(b-a)/gr
        
    fun_min = (a+b)/2

    return fun_min

def test_gss():
    fun1 = lambda x: x**2
    fun2 = lambda x: math.cos(x)
    fun3 = lambda x: math.exp(x)

    assert abs(gss(fun1,-5,5)) < 1e-5
    assert abs(gss(fun2,-5,5) - math.pi) < 1e-5
    assert abs(gss(fun3,-5,5) + 5) < 2e-5

if __name__ == "__main__":
    test_gss()
