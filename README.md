# Event-triggered Decentralized Data Fusion

This repository provides a library of Python and ROS code implementing event-trigged decentralized data fusion (ET-DDF) algorithms, which enable decentralized cooperative localization and tracking at significantly reduced communication overheads. See [[1]](#References) and [[2]](#References) for more information about the algorithms themselves.

In addition, this repository provides a solution to windowed event-triggering filtering using an algorithm called Delta Tiering. The windowed communication problem can be defined as a situation in which there is a certain time period to communicate (share measurements in the case of ETDDF) but communication is not permitted outside of the time window. A buffer to store desired communications (measurements) is therefore necessary during the periods where communication is not permitted. Once the communication window opens, this buffer of communications is pulled and sent to the other asset. The Delta Tiering algorithm optimizes information sent to the other asset by maintaining multiple buffers and at pull time selecting the one with the most information.

The basic filter in the repository is the ETFilter (Event Triggered Filter). This class implements an Extended Kalman Filter. For non-windowed or more traditional asynchronous communication the Asset class is provided to manage common etfilters with other agents and implicit message generation. For windowed communication, the LedgerFilter class is provided that tracks all measurements, covariance intersects, and control inputs that the filter inputs. When a packet is received these ledgers are combined with the information in the packet from the other asset and then rolled forward to the present time fusing all of the combined information. The LedgerFiler similarly maintains a buffer (under the hood multiple buffers) of measurements that can be pulled at any time and transmitted to another asset.


These algorithms, and their implementations in this repository are currently in use as part of DARPA sprinter project OFFSET, as well as a collaboration between Orbit Logic, the University of Colorado Boulder, the University of California San Diego, and the Office of Naval Research called MinAu (Minimal Autonomy).

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


## References

1) I. Loefgren, N. Ahmed, E. W. Frew,  C. Heckman, and S. Humbert. Scalable Event-Triggered data fusion for autonomous cooperative swarm localization. In 2019 22nd International Conference on Information Fusion (FUSION) (FUSION 2019), Ottawa, Canada, July 2019. 
2) M. Ouimet, D. Iglesias, N. Ahmed, and S. Martı́nez, “Cooperative Robot
Localization Using Event-Triggered Estimation,” Journal of Aerospace
Information Systems, no. 7, pp. 427–449, jul.
