# Event-triggered Decentralized Data Fusion

This repository provides a library of Python and ROS code implementing event-trigged decentralized data fusion (ET-DDF) algorithms, which enable decentralized cooperative localization and tracking at significantly reduced communication overheads. See [[1]](#References) and [[2]](#References) for more information about the algorithms themselves.

In addition, this repository provides a solution to windowed event-triggering filtering using an algorithm called Delta Tiering. The windowed communication problem can be defined as a situation in which there is a certain time period to communicate (share measurements in the case of ETDDF) but communication is not permitted outside of the time window. A buffer to store desired communications (measurements) is therefore necessary during the periods where communication is not permitted. Once the communication window opens, this buffer of communications is pulled and sent to the other asset. The Delta Tiering algorithm optimizes information sent to the other asset by maintaining multiple buffers and at pull time selecting the one with the most information.

The basic filter in the repository is the ETFilter (Event Triggered Filter). This class implements an Extended Kalman Filter. For non-windowed or more traditional asynchronous communication the Asset class is provided to manage common etfilters with other agents and implicit message generation. For windowed communication, the LedgerFilter class is provided that tracks all measurements, covariance intersects, and control inputs that the filter inputs. When a packet is received these ledgers are combined with the information in the packet from the other asset and then rolled forward to the present time fusing all of the combined information. The LedgerFiler similarly maintains a buffer (under the hood multiple buffers) of measurements that can be pulled at any time and transmitted to another asset.


These algorithms, and their implementations in this repository are currently in use as part of DARPA sprinter project OFFSET, as well as a collaboration between Orbit Logic, the University of Colorado Boulder, the University of California San Diego, and the Office of Naval Research called MinAu (Minimal Autonomy).

The etddf_node.py is implemented in **ENU** coordinate frame for the minau project.

## Running Instructions
After building the package, the etddf node can be roslaunched.
```
roslaunch etddf etddf.launch
```

## ROS Interfaces
### Esimates
ETDDF's estimates of assets can be accessed all at once or individually per asset.
The topic for the "Network Estimate" or the estimates of all assets in an array is published on
```
etddf/estimate/network
```
Individual asset estimates (with name <asset_name>) using the traditional Odometry message are published on
```
etddf/estimate/<asset_name>
```
### Buffer Pulling
ETDDF's optimal buffer can be pulled with the following service
```
etddf/get_measurement_package
```
NOTE: The ETDDF instance on the other asset expects the buffer in the same form (MeasurementPackage) though the order of the measurements need not be maintainted (as long as timestamps are preserved).

Each MeasurementPackage must have Measurements with the following meas_types (Measurement.meas_type)
- meas_type="final_time" indicating the final time measurements were considered for this package
- a measurement stream start, a "bookstart". This can be indicated by any actual measurement OR by a meas_type="*_bookstart". Either of these tell ETDDF to start filling in implicit measurements after this time
- Optionally, a measurement stream stop, a "bookend". These indicate that a measurement stream stopped; for example, an asset being no longer in view of the asset's sonar would generate a "sonar_x_bookend" measurement. These are indicated by meas_type="*_bookend". If the measuremnt stream continues through the end of "final_time" a bookend is not needed.

The currently supported measurement types (Measurement.meas_type) are
- depth
- modem_range (asset-to-asset measurement differs from surface-to-asset measurement by value of Measurement.global_pose. It is global_pose=[], empty list, for asset to asset range)
- modem_azimuth (see note on modem_range)
- dvl_x
- dvl_y
- sonar_x
- sonar_y
- sonar_z

### Buffer Receival
The ETDDF node can receive a MeasurementPackage on the topic:
```
etddf/packages_in
```

## Configuration
Measurement configuration can be found in config/measurements.py.

Filter and general Delta Tiering configurations can be adjusted in config/etddf.yaml.


## Repeat Tester
Sets up scenario where 2 blue_rovs are patrolling an area, searching for the red_actor as it passes through the space. During each test the red_actor will pass all the way through the area. At the end of the test the blue_rovs and red_actor will respawn and the test will be rerun with different settings depending on the settings specified in the csv.

Configuration for the repeat tester can be found in config/repeat.csv.

Any Configuration Group ending in _dep will be the same for all tests.

List of configuration settings:

- Test_Group_Name: Name for the group of tests you are doing. Must be different than what you have done before so repeat tester can make another directory.
- Test_Name: Name of the specfic test. 
- Number_Tests_dep: Number of tests you will do with each set of settings specified by each specific test.
- Map_Dim_x_dep: How far the ends of the search rectangle are from x axis
- Map_Dim_y_dep: How far the ends of the search rectangle are from y axis
- DVL_Data: True/False. Setting to turn on or off dvl data
- Lawn_Mode: True/False. Setting to turn on and off lawn mode. If lawn mode is off it wil use the bayes filter to search the modes of the space.
- Red_Vel_dep: The speed of the red_actor.
- Blue_Vel_dep: The speed of the blue_actor.
- Custody_Dist: How close the blue_rovs have to get to the red asset before they just look at it.

How to use repeat tester:

1) Set up tests using the csv to control the settings of the test.
2) Start up oceans.launch
```
roslaunch etddf scenario.sitl.ocean.launch
```
3) Press play in Gazebo
4) Launch repeat test
```
roslaunch etddf repeat.launch
```

All the data will be baged in the folder scripts/data/(Test_Group_Name)/(Test_Name).bag
The topics that are baged are:
- /bluerov2_3/pose_gt
- /bluerov2_4/pose_gt
- /red_actor_1/pose_gt
- /bluerov2_3/etddf/estimate/network
- /bluerov2_4/etddf/estimate/network
- /bluerov2_3/strapdown/estimate
- /bluerov2_4/strapdown/estimate
- /bluerov2_3/sonar_processing/target_list 
- /bluerov2_4/sonar_processing/target_list

To analyze the repeat tester data do:
```
./repeat_data_analyzer.py data/Test_Group_Name x_dim y_dim
```
Where Test_Group_Name is the name corresponding the the Test_Group_Name in the repeat.csv. The x_dim and y_dim are the dimensions of the space as specified in the repeat.csv also.
This will graph the network error for each test, as well as the time to capture, percent of time tracked and percent found as well as make a csv within the data/Test_Group_Name directory of all the relevant data.

If you want all the graphs for each test setting subplotted on one graph you can do:
```
./graph_groups.py data/Test_Group_Name
```
This will use all the data in the Test_Group_Name.csv to make these plots.


## References

1) I. Loefgren, N. Ahmed, E. W. Frew,  C. Heckman, and S. Humbert. Scalable Event-Triggered data fusion for autonomous cooperative swarm localization. In 2019 22nd International Conference on Information Fusion (FUSION) (FUSION 2019), Ottawa, Canada, July 2019. 
2) M. Ouimet, D. Iglesias, N. Ahmed, and S. Martı́nez, “Cooperative Robot
Localization Using Event-Triggered Estimation,” Journal of Aerospace
Information Systems, no. 7, pp. 427–449, jul.
