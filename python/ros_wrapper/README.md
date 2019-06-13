# OFFSET ROS ET-DDF

This ROS package, `etddf_ros`, integrates the OFFSET event-triggered decentralized data fusion (ET-DDF) library into a ROS framework for use in simulations and on hardware. Below are run instructions, as well as an overview of the architecture of the system.

This package was developed as part of the DARPA OFFSET project.

_Note: This software is currently configured to use linear dynamics and linear absolute and relative position measurements. All agents estimates and measurements are assumed to be in an ENU frame._

---

## Installation Instructions

In order to use the `etddf_ros` package, first clone the `etddf` repository to some location on your machine, `<etddf repo location>`. This will download both the ROS package as well as the core ET-DDF library.  

It is highly recommended to install the ET-DDF code in a virtual environment to preclude potential issues with paths and package dependencies. This was the method with which the library, and this ROS package was tested. Make sure the venv is using python2!

Once you've cloned the `etddf` repository, install the core ET-DDF library by navigating to the python code and running the setup script using pip:

```
$ cd <etddf repo location>/python/
$ pip2 install -e . -r requirements.txt
```

This installs the core ET-DDF modules into your path so python will be able to find them. _Note: make sure you use pip2 to install everything, as ROS 1 requires python2, and running everything in python2 will reduce headaches._

The following packages are installed:
- pyyaml
- numpy
- scipy
- matplotlib
- pudb
- catkin_pkg
- empy
- rospkg

Next, create a symbolic link from the ROS package, located in the `etddf_ros` folder, to a catkin workspace, then build your catkin workspace, and source it:

```
$ ln -s <etddf repo location>/python/offset_ros/ <path to a catkin workspace>/src/
$ cd <path to catkin_workspace> && catkin_make && source devel/setup.bash
```

This will build your catkin workspace, and generate the necessary message types to use the package, as well as make sure your workspace is in your ROS path.

---

## Run Instructions

### Launching full simulation

To launch a full simulation, including simulated measurements, control input and simulated vehicles (represented as point masses):

```
$ roslaunch offset_etddf offset_etddf.launch
```

### Launching agent instance

To launch an instance of ET-DDF, use the `agent.launch` launch file. This will launch an ET-DDF node, a comms module node, and, if set, nodes for simulating sensors measurements ang generating control input.

```
$ roslaunch offset_etddf agent.launch id:=[id int value]
```

The arguments are as follows:

- id: integer to uniquely identify an instance of ET-DDF -- __Required__
    - Note: it is important that these ids correspond with the ids in the config files, otherwise, messages will not be sent to the correct agents, or might not be sent at all
- agent_name: name used for namespacing the ET-DDF instance -- _default:_ "agent_\<id>"
- log_level: level of ROS log messages will be displayed to the console -- _default:_ "INFO" 
    - options are: "DEBUG", "INFO", "WARN", "ERROR", "FATAL"
- sensors: boolean to simulate sensor measurements -- _default:_ false
- planner: boolean to simulate control input -- _default:_ false


### Viewing Performance

To view the quality of estimates, you can use the `etddf_ros/helpers/viz.py` node. This node is currently only configured for a 4 agent case, but can be used to confirm things are functioning properly. 

This node uses simuation ground truth pose data to compute estimate error and error magnitude. These can be viewed in a terminal, or using the `rqt_plot` utility.

---

## Configuration Instructions

### Config Files

There are two main config files used for running ET-DDF in ROS. Both are located in the `config/` folder of the `etddf_etddf` package.

The `ros_agent_config.yaml` config file is used to configure ET-DDF instances, including setting connections, ET thresholding, CI thresholding, and update rate for the ET-DDF instance.

_Note: All connections for the entire network must be specified in the above config file._
Connections are specified as a nested list, where index corresponds to the agent, and the list located at that index are the ids of the agent's connections. _E.g. [[1],[0]] represents agent 0 being connected to agent 1 and vice versa._

The `points.yaml` config file is used to configure simulation parameters, if a simulation is being run, such as starting location, planner type, and simulated sensor noise and update rate.

The current configuration is stable and performs well, but if the CI threshold is set too low, and the update rate too high or too low, ET-DDF instances will struggle to keep up, start dropping messages, and diverging.


### Configuring Topic Names

To configure ET-DDF for use in a system, there are a few topics that need to be configured. 

First are the topics for the sensors each ET-DDF instance uses, found in `etddf_ros/src/agent_wrapper.py`. The default topic names are those that are used when simulating sensor measurements.

<!-- Second is making sure that all ET-DDF instances are namespaced correctly.  -->

---

## Architecture Overview

The `etddf_ros` package provides a wrapper around the `etddf` ET-DDF library, as well as the tools to manage, convert, and move sensor, and between-agent messages through the system.

Below is a block diagram of a high-level view of the system:

<!-- ![system block diagram](docs/images/sys_diagram.png) -->
<!-- Format: ![System diagram] -->
<img src="docs/images/sys_diagram.png" alt="System diagram" width="600"/>

### ROS Nodes

There are two nodes that are launched for each use of `agent.launch` to wrap the ET-DDF library:

- agent_wrapper
- comms_module

The __agent_wrapper__ node is the actual wrapper for the ET-DDF library, and runs at a rate defined in the `ros_agent_config.yaml` config file. Running asynchronously are the callbacks to queue messages for local sensor measurements, and thresholded measurement messages from other agents.

The update loop performs the following steps:

1. Empty the local measurement queue, and convert messages from ROS to Python messages
2. Update filters and threshold measurements using the _process_local_measurements()_ function
3. Convert the thresholded measurements to ROS messages and publish to the comms module, to be sent to connected agents
4. Empty the received (from other agents) measurement queue, and convert from ROS to Python messages
5. Update filters using the _process_received_measurements()_ function
6. Check if covariance intersection (CI) needs to be performed, and if so, generate and send CI requests to other agents, queuing responses
7. Empty the CI state message queue, convert to Python messages, and perform CI with each state message.

See the README for the `etddf` library to read more about the architecture and underlying algorithms for ET-DDF.

The __comms_module__ node simply abstracts communication with the outside world by allowing the __agent_wrapper__ to send all messages to one topic, from which the module sends each message to the appropriate destination agent's __comms_module__.

There are also two additional nodes available for use in simulations:

- publish_sensors
- planner

The __publish_sensors__ node uses ground truth pose data for a agent to create simulated sensor measurements to be used for ET-DDF. The __planner__ node generates control input in the form of twist messages for simulated agents (represented by point masses). These two nodes allow for a full ET-DDF simulation without additional hardware or simulators.

---

## ROS 2 Compatibility

In its current state, the `etddf_ros` package is implemented using __ROS 1__. However for use in a larger system running __ROS 2__, there are only a few locations where a message bridge is needed. These are the following:

- Sensor measurements
- _[if desired]_ publishing local estimates for use outside of the et-ddf namespace