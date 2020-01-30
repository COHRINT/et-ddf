# Python ET-DDF

This folder contains the python version of the ET-DDF library. This is the version of the code under active development, and it is highly recommended that this is the version of the library used, as the Matlab version has no received updates since v1.0.

## Library Structure

The core of the ET-DDF library is located in the `etddf/` folder, including code to run a pure python ET-DDF simulation. See the README in the `etddf/` folder for more information about core library functions.

The config file for a pure python simulation is located in `config/`, and is a YAML file that specifies all parameters for the simulation.

The `ros_wrapper/` folder contains the code to wrap the ET-DDF library for use in ROS, allowing for the use of ET-DDF in hardware applications/.

## Installation

To install the python ET-DDF library, first create or active a virtual environment in which you want to install the code. A virtual environment is not required to run the code, but greatly simplifies management of package versions and ensures that there aren't version conflicts with system installs of certain packages.

### Python Version Compatibility

The ET-DDF library has been tested against the following versions:

- 2.7
- 3.5
- 3.6 (limited use thus far)

There is little reason to believe newer versions of python will not be compatible, but this can not be guaranteed. Depending on if you are planning on using the code in a pure python simulation context or in conjunction with ROS, you can choose to install with python 2.7 or python 3. It is suggested that you use python 3 if you do not have plans for using ROS, and can mostly likely use python 3 regardless.

If you want to use the python 2.7 version, in a terminal run the following to install the library and dependencies

```
$ pip2 install -e . -r requirements.txt
```

This will install an editable version of the ET-DDF package, so you don't have to reinstall every time you make a change to the code. For python 3 run

```
$ pip3 install -e . -r requirements.txt
```

Once you have installed the library you are ready to go! See the README in `etddf/` for pure python simulation information and instructions, and the README in `ros_wrapper/` for information and instructions about the ROS version of the code.