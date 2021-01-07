#!/usr/bin/env python
from __future__ import division
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from etddf.msg import Measurement, MeasurementPackage
from etddf.srv import GetMeasurementPackage
import rospy
import numpy as np
from cuprint.cuprint import CUPrint
from cuquantization.quantize import measPkg2Bytes, bytes2MeasPkg

"""
This node serves as a snub for the seatrac modem
Generates:
    - range & azimuth to assets
    - manages communication of packets to other assets
"""
COMMS_ACTION_INDEX = 0
COMMS_TIME_INDEX = 1

RANGE_SD = 0.1
AZIMUTH_SD = 5

GLOBAL_POSE = [0,0,0,0]

class SeatracSnub:

    def __init__(self, robot_names):
        self.poses = {}
        self.poses["surface"] = Pose() # Assume surface beacon located (0,0,0)
        self.poses["surface"].orientation.w = 1
        for r in robot_names:
            rospy.Subscriber(r + "/pose_gt", Odometry, callback = self.pose_callback, callback_args = r)
            rospy.wait_for_message(r + "/pose_gt", Odometry)

    def pose_callback(self, msg, robot_name):
        self.poses[robot_name] = msg.pose.pose

if __name__ == "__main__":
    rospy.init_node("seatrac_snub")
    cuprint = CUPrint(rospy.get_name())

    # [comms_type, time_taken]
    # comm_scheme = [["ping_surface_to_bluerov2_3", 3],["broadcast_surface",4]]
    # comm_scheme = [["ping_surface_to_bluerov2_3", 1],["broadcast_surface",1]]
    comm_scheme = [["ping_surface_to_bluerov2_3", 3], ["ping_surface_to_bluerov2_4", 3], ["ping_surface_to_bluerov2_5", 3],
    ["broadcast_surface",3]]#, ["broadcast_bluerov2_3",4], ["broadcast_bluerov2_4",4]]

    asset_landmark_dict = {"surface" : 0, "bluerov2_3":1, "bluerov2_4" : 2, "bluerov2_5" : 3}

    assets = ["bluerov2_3", "bluerov2_4", "bluerov2_5"]
    meas_pkg_pub_dict = {}
    for a in assets:
        meas_pkg_pub_dict[a] = rospy.Publisher(a + "/etddf/packages_in", MeasurementPackage, queue_size=10)
    seasnub = SeatracSnub(assets)

    curr_index = 0

    surface_meas_pkg = MeasurementPackage()
    latest_meas_pkg = MeasurementPackage()

    cuprint("Loaded")

    while not rospy.is_shutdown():
        curr_comms = comm_scheme[curr_index]
        curr_action = curr_comms[COMMS_ACTION_INDEX]
        t = rospy.get_rostime()

        if "ping" in curr_action:
            new_str = curr_action[len("ping_"):]
            action_executed_by = new_str[ :new_str.index("_to_")]
            measured_asset = new_str[new_str.index("_to_") + len("_to_"):]

            cuprint(action_executed_by + " pinging " + measured_asset)
            
            action_executed_by_pose = seasnub.poses[action_executed_by]
            measured_asset_pose = seasnub.poses[measured_asset]

        
            diff_x =measured_asset_pose.position.x - action_executed_by_pose.position.x
            diff_y =measured_asset_pose.position.y - action_executed_by_pose.position.y
            diff_z =measured_asset_pose.position.z - action_executed_by_pose.position.z
            dist = np.linalg.norm([diff_x, diff_y, diff_z]) + np.random.normal(0, RANGE_SD)
            range_meas = Measurement("modem_range", t, action_executed_by, measured_asset, dist, RANGE_SD**2, GLOBAL_POSE)
            
            if "surface" in curr_action:
                latest_meas_pkg.src_asset = action_executed_by
                surface_meas_pkg.measurements.append(range_meas)
                # include azimuth
                # diff_x, diff_y = diff_y, diff_x # transform to NED in gazebo
                # az_sd = ( 15*np.random.uniform() + 30 ) * (np.pi/180)
                ang = np.arctan2(diff_y, diff_x) #+ np.random.normal(0, az_sd)
                ang_deg = np.rad2deg(ang) + np.random.normal(0, AZIMUTH_SD)
                az_meas = Measurement("modem_azimuth", t, action_executed_by, measured_asset, ang_deg, AZIMUTH_SD**2, GLOBAL_POSE)
                surface_meas_pkg.measurements.append(az_meas)
            else:
                latest_meas_pkg.src_asset = action_executed_by
                latest_meas_pkg.measrements.append(range_meas)

        elif "broadcast" in curr_action:
            if "surface" in curr_action:
                surface_meas_pkg.src_asset = "surface"
                rospy.loginfo("surface broadcasting")

                # bytes_ = measPkg2Bytes(surface_meas_pkg, asset_landmark_dict)
                # surface_meas_pkg = bytes2MeasPkg(bytes_, 0.0, asset_landmark_dict, GLOBAL_POSE)

                for asset_key in meas_pkg_pub_dict.keys():
                    meas_pkg_pub_dict[asset_key].publish(surface_meas_pkg)
                surface_meas_pkg = MeasurementPackage()
            else:
                agent = curr_action[len("broadcast_"):]
                rospy.wait_for_service(agent + "/etddf/get_measurement_package")
                gmp = rospy.ServiceProxy(agent + "/etddf/get_measurement_package", GetMeasurementPackage)
                try:
                    meas_pkg = gmp().meas_pkg
                    print('[Seatrac Snub] delivering buffer: ')
                    print(meas_pkg)
                    for asset_key in meas_pkg_pub_dict.keys():
                        if asset_key != agent:
                            print("publishing to: " + asset_key)
                            rospy.sleep(0.1)
                            meas_pkg_pub_dict[asset_key].publish(meas_pkg)
                            rospy.sleep(0.1)
                except rospy.ServiceException as e:
                    print(e)

        curr_index = (curr_index + 1) % len(comm_scheme)
        rospy.sleep( curr_comms[COMMS_TIME_INDEX] )
        