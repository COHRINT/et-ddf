import rospy

from etddf.msg import MeasurementPackage, Measurement
import numpy as np
import sys
import rospy

"""
Structure of Pkg (28 bytes)
1 Byte: Delta Multiplier
1 Byte: header
1 Byte: time
(if necessary)
1 Byte: data

"""

# Asset landmark dict is asset/landmark --> int 0-16

################ API FUNCTIONS ################
def measPkg2Bytes(meas_pkg, asset_landmark_dict):
    byte_string = []

    # Delta Multiplier
    dm_index = delta_multiplier_options.index(meas_pkg.delta_multiplier)
    if dm_index < 0:
        print("Could not locat delta multiplier: " + str(meas_pkg.delta_multiplier) + " in " + str(delta_multiplier_options))
        sys.exit(-1)
    else:
        src_asset_code = asset_landmark_dict[meas_pkg.src_asset]
        primary_header = dm_index << 4 | src_asset_code
        byte_string.append(primary_header)

    # Measurements
    present_time = rospy.get_rostime()
    for meas in meas_pkg.measurements:
        header = HEADERS[meas.meas_type]
        header2 = 0
        for mwa in MEASUREMENTS_WITH_AGENTS:
            if mwa in meas.meas_type:
                header2 = asset_landmark_dict[meas.measured_asset]
                break
        timestamp = (present_time - meas.stamp).secs
        data_bin = 0

        # Compression
        if meas.meas_type == "depth":
            # Range [-3,0]
            # 256 bins
            bin_per_meter = 256 / -3.0
            data_bin = int(meas.data * bin_per_meter)
        elif meas.meas_type in ["sonar_x", "sonar_y"]:
            # Range [-10,10] -> [0, 20] Shift range for convenience
            # 256 bins
            bin_per_meter = 256 / 20.0
            data = meas.data + 10.0 # Shift the sonar range to be between 0 and 20
            data_bin = int(data * bin_per_meter)
        elif meas.meas_type == "modem_range":
            # Range [0, 20]
            # 256 bins
            bin_per_meter = 256 / 20.0
            data_bin = int(meas.data * bin_per_meter)
        elif meas.meas_type == "modem_azimuth":
            # Range [0, 360]
            # 256 bins
            bin_per_meter = 256 / 360.0

            while meas.data > 360:
                meas.data -= 360.0
            while meas.data <= 0:
                meas.data += 360.0
            
            data_bin = int(meas.data * bin_per_meter)

        # Append to bytestring
        full_header = header << 4 | header2
        byte_string.append(full_header)
        byte_string.append(timestamp)
        if "book" not in meas.meas_type:
            byte_string.append(data_bin)

    return byte_string

def bytes2MeasPkg(byte_arr, transmission_time, asset_landmark_dict, global_pose):

    mp = MeasurementPackage()

    primary_header = byte_arr[0]
    src_asset_code = primary_header & 15
    mp.src_asset = asset_landmark_dict.keys()[asset_landmark_dict.values().index( src_asset_code )]
    dm_index = ( primary_header & ( 15 << 4 ) ) >> 4
    mp.delta_multiplier = delta_multiplier_options[dm_index]

    
    index = 1
    present_time = rospy.get_rostime()
    while index < len(byte_arr):
        header = byte_arr[index]
        meas_type = HEADERS.keys()[HEADERS.values().index( (header & (15 << 4)) >> 4 )]
        header2 = header & 15
        index += 1

        timestamp = rospy.Time( (present_time.secs - transmission_time) - byte_arr[index] )

        index += 1

        if "book" not in meas_type:
            data_bin = byte_arr[index]
            data = 0
            # Compression
            if meas_type == "depth":
                bin_per_meter = 256 / -3.0
                data = data_bin / bin_per_meter
            elif meas_type in ["sonar_x", "sonar_y"]:
                bin_per_meter = 256 / 20.0
                data = data_bin / bin_per_meter - 10.0
            elif meas_type == "modem_range":
                bin_per_meter = 256 / 20.0
                data = data_bin / bin_per_meter
            elif meas_type == "modem_azimuth":
                bin_per_meter = 256 / 360.0
                data = data_bin / bin_per_meter
                data = np.mod( data + 180, 360) - 180 # -180 to 180
            measured_agent = ""
            for mwa in MEASUREMENTS_WITH_AGENTS:
                if mwa in meas_type:
                    measured_agent = asset_landmark_dict.keys()[asset_landmark_dict.values().index( header2 )]
                    break
            m = Measurement(meas_type, timestamp, mp.src_asset, measured_agent, data, 0.0, global_pose)
            mp.measurements.append(m)
            index += 1
        else:
            measured_agent = ""
            for mwa in MEASUREMENTS_WITH_AGENTS:
                if mwa in meas_type:
                    measured_agent = asset_landmark_dict.keys()[asset_landmark_dict.values().index( header2 )]
                    break
            m = Measurement(meas_type, timestamp, mp.src_asset, measured_agent, 0.0, 0.0, global_pose)
            mp.measurements.append(m)

    return mp

################ PRIVATE MEMBERS ################
MEASUREMENTS_WITH_AGENTS = ["sonar", "modem"]

HEADERS = {
    'empty' : 0,
    'depth' : 1,
    'depth_bookstart' : 2,
    'depth_bookend' : 3,
    'sonar_x' : 4,
    'sonar_x_bookstart' : 5,
    'sonar_x_bookend' : 6,
    'sonar_y' : 7,
    'sonar_y_bookstart' : 8,
    'sonar_y_bookend' : 9,
    'modem_range' : 10,
    'modem_azimuth' : 11
}

delta_multiplier_options = list(np.arange(0,11,1))

if __name__ == "__main__":
    # Create measurement package
    rospy.init_node("test_quantize")

    global_pose = [1,2,3,4]

    asset_landmark_dict = {"bluerov2_3" : 0, 
        "bluerov2_4" : 1, "landmark_pole1" :2, "surface" : 4
    }

    print('############ TEST 1 #################')
    # Test compression and decompression
    mp = MeasurementPackage()
    mp.src_asset = "surface"
    mp.delta_multiplier = 1.0
    t = rospy.get_rostime()
    m = Measurement("modem_range", t, mp.src_asset, "bluerov2_3", 3.65, 0.5, global_pose)
    m2 = Measurement("modem_azimuth", t, mp.src_asset, "bluerov2_3", -65.72, 0.5, global_pose)
    m3 = Measurement("modem_range", t, mp.src_asset, "bluerov2_4", 7.8, 0.5, global_pose)
    m4 = Measurement("modem_azimuth", t, mp.src_asset, "bluerov2_4", 23.0, 0.5, global_pose)
    mp.measurements.append(m)
    mp.measurements.append(m2)
    mp.measurements.append(m3)
    mp.measurements.append(m4)

    
    # print(mp)
    bytes_ = measPkg2Bytes(mp, asset_landmark_dict)
    print(bytes_)
    mp_return = bytes2MeasPkg(bytes_, 2, asset_landmark_dict, global_pose)
    # print(mp_return)

    print('############ TEST 2 #################')
    mp = MeasurementPackage()
    mp.src_asset = "bluerov2_3"
    mp.delta_multiplier = 5.0

    t = rospy.get_rostime()
    m = Measurement("sonar_x", t, mp.src_asset, "bluerov2_4", 2.1, 0.5, [])
    m2 = Measurement("sonar_y", t, mp.src_asset, "bluerov2_4", -6.5, 0.5, [])
    m3 = Measurement("depth", t, mp.src_asset, "", -1.3, 0.5, [])
    m4 = Measurement("sonar_x_bookend", rospy.get_rostime(), mp.src_asset, "bluerov2_4", 0.0, 0.5, [])
    m5 = Measurement("depth_bookend", rospy.get_rostime(), mp.src_asset, "", 0.0, 0.5, [])
    m6 = Measurement("sonar_y_bookstart", rospy.get_rostime(), mp.src_asset, "landmark_pole1", 0.0, 0.5, [])

    mp.measurements.append(m)
    mp.measurements.append(m2)
    mp.measurements.append(m3)
    mp.measurements.append(m4)
    mp.measurements.append(m5)
    mp.measurements.append(m6)
    print(mp)
    bytes_ = measPkg2Bytes(mp, asset_landmark_dict)
    print(bytes_)
    mp_return = bytes2MeasPkg(bytes_, 2, asset_landmark_dict, global_pose)
    print(mp_return)
