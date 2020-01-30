# Event-triggered Decentralized Data Fusion

This repository provides a library of Matlab and Python code implementing event-trigged decentralized data fusion (ET-DDF) algorithms, which enable decentralized cooperative localization and tracking at significantly reduced communication overheads. See [[1]](#References) and [[2]](#References) for more information about the algorithms themselves.

These algorithms, and their implementations in this repository are currently in use as part of DARPA sprinter project OFFSET, as well as a collaboration between Orbit Logic, the University of Colorado Boulder, the University of California San Diego, and the Office of Naval Research called MinAu.


## Python Simulation

The Python version of the ET-DDF library provides an implementation of a linear event-triggered Kalman filter, with an event-triggered extended Kalman filter in progress. It also provides a class to contain and interact with the filters needed for ET-DDF, including handling messages, all inside a simulation framework.

See the Python [ET-DDF README](python/etddf/README.md) for more information on running simulations in Python.


## Python & ROS

There is ROS package, `etddf_ros` that wraps the Python ET-DDF library described above. The package provides an interface for handling sensor measurements and inter-vechicle communications as ROS messages, allowing the algorithms to be run on an acutal system.

To use the ROS ET-DDF package, make sure you have ROS installed on your machine, then see the [ROS ET-DDF README](python/ros_wrapper/README.md). *Note: the ROS package was developed in and tested with ROS Kinetic and Melodic, however will likely work with other distributions* 

## Matlab Simulation

The project was originally implemented in Matlab, and has not received major updates since v1.0. Please see the README located in the `matlab/` directory for instructions on using and particulars about the Matlab version of the code.

### __It is recommended to use the python or python/ROS versions of the code.__


## References

1) I. Loefgren, N. Ahmed, E. W. Frew,  C. Heckman, and S. Humbert. Scalable Event-Triggered data fusion for autonomous cooperative swarm localization. In 2019 22nd International Conference on Information Fusion (FUSION) (FUSION 2019), Ottawa, Canada, July 2019. 
2) M. Ouimet, D. Iglesias, N. Ahmed, and S. Martı́nez, “Cooperative Robot
Localization Using Event-Triggered Estimation,” Journal of Aerospace
Information Systems, no. 7, pp. 427–449, jul.
