#!/usr/bin/env python

import sys
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from etddf_ros2_msgs.msg import GpsMeasurement, LinrelMeasurement
from geometry_msgs.msg import Pose
from decimal import Decimal
import numpy as np
# import tf2
from functools import partial

from etddf.helpers.config_handling import load_config
from ament_index_python.packages import get_package_share_directory

"""
Publishes
- depth measurements for current robot
- Measurement.msg for relative positionings to other robots

range: res 0.1m
azimuth: 0-360 (res 0.1 deg)
elevation: -90 to 90 (res 0.1 deg)
fit error: values toward 0 are more confident, values 2-3 poor fits (res 0.01)
"""

class SensorPub:

    auvs = {}
    
    def __init__(self, name, active_robots, cfg):

        rclpy.init()
        self.node = rclpy.create_node('sensors')

        self.name = name
        for rob in active_robots:
            self.node.create_subscription(Odometry, '/' + rob + '/pose_gt', partial(self.pose_callback, rob))
            self.auvs[rob] = None
        self.pose = None

        # Publishers
        # self.depth_pub = rospy.Publisher('depth', depthMeasurement, queue_size=10)
        # self.usbl_pub = rospy.Publisher('usbl', usblMeasurement, queue_size=10)
        self.gps_pub = self.node.create_publisher(GpsMeasurement, 'gps')
        self.linrel_pub = self.node.create_publisher(LinrelMeasurement, 'lin_rel')

        # Timer callbacks
        # depth_rate = 1 / float(cfg['/sensors/depth_pub_rate'], 50))
        # usbl_rate = 1 / float(cfg['/sensors/usbl_pub_rate'], 0.5))
        # self.depth_timer = rospy.Timer(rospy.Duration(depth_rate), self.depth_callback)
        # self.usbl_timer = rospy.Timer(rospy.Duration(usbl_rate), self.usbl_callback)
        gps_rate = 1 / float(cfg['sensors']['gps_pub_rate'])
        linrel_rate = 1 / float(cfg['sensors']['lin_rel_pub_rate'])
        self.gps_timer = self.node.create_timer(1/gps_rate, self.gps_callback)
        self.linrel_timer = self.node.create_timer(1/linrel_rate, self.linrel_callback)

        # Noise
        self.noise = cfg['sensors']['noise']
        # self.depth_noise = float(cfg['/sensors/depth_noise', 0.03))
        # self.angular_noise = float(cfg['/sensors/angular_noise',1))
        # self.distance_noise = float(cfg['/sensors/distance_noise', 1.0))
        self.gps_noise = float(cfg['sensors']['gps_noise'])
        self.lin_rel_noise = float(cfg['sensors']['lin_rel_noise'])

        # Resolutions
        # self.dist_res = int(cfg['/sensors/distance_res', 1))
        # self.angular_res = int(cfg['/sensors/angular_res', 1))
        # self.depth_res = int(cfg['/sensors/depth_res', 2))
        # self.pos_res = int(cfg['/sensors/positioning_res',1))
        
        self.node.get_logger().info("Sensor pub for " + name + " initialized")
        self.gpsSeq = 0
        self.linrelSeq = 0

    def pose_callback(self, name, msg):
        # topic = msg._connection_header['topic']
        topic = name
        auv = None
        for auv_name in self.auvs.keys():
            if auv_name in topic:
                auv = auv_name
                break
        if auv == self.name:
            self.pose = msg.pose.pose
        else:
            self.auvs[auv] = msg

    def depth_callback(self, msg):
        if self.pose == None:
            return
        depth = self.pose.position.z
        depth = self.insert_noise(depth, self.depth_noise)
        meas = DepthMeasurement()
        meas.data = round(depth, self.depth_res)
        meas.header.seq = self.depthSeq
        meas.header.stamp = self.node.get_clock().now()
        # meas.header.frame_id = TODO proper reference frame
        self.depth_pub.publish(meas)
        self.depthSeq += 1
        
    def usbl_callback(self, msg):
        meas = UsblMeasurement()
        for auv in self.auvs:
            if self.auvs[auv] == None or self.pose == None:
                continue

            # Measurement
            meas.header.seq = self.usblSeq
            meas.header.stamp = self.node.get_clock().now()
            # meas.header.frame_id = TODO proper reference frame
            meas.robot_measured = auv
            _range = self.get_distance(self.pose.position, self.auvs[auv].pose.pose.position)
            _range = self.insert_noise(_range, self.distance_noise)
            az, elev = self.get_bearing(self.pose,self.auvs[auv].pose.pose.position)
            az = self.insert_noise(az, self.angular_noise)
            elev = self.insert_noise(elev, self.angular_noise)
            # rospy.loginfo("Publishing dist to " + auv)       
            meas.range = round(_range, self.dist_res)
            meas.azimuth = round(az, self.angular_res)
            meas.elevation = round(elev, self.angular_res)
            meas.fit_error = 0
            self.usbl_pub.publish(meas)
            self.usblSeq += 1

    def gps_callback(self):
        if self.pose == None:
            return
        x = self.pose.position.x
        y = self.pose.position.y
        z = self.pose.position.z
        x = self.insert_noise(x, self.gps_noise)
        y = self.insert_noise(y, self.gps_noise)
        z = self.insert_noise(z, self.gps_noise)
        meas = GpsMeasurement()
        # meas.data = round(depth, self.depth_res)

        meas.x = x
        meas.y = y
        meas.z = z

        # meas.header.seq = self.gpsSeq
        meas.header.stamp.sec = int(self.node.get_clock().now().seconds_nanoseconds()[0])
        meas.header.stamp.nanosec = int(self.node.get_clock().now().seconds_nanoseconds()[1])
        # meas.header.frame_id = TODO proper reference frame
        self.gps_pub.publish(meas)
        self.gpsSeq += 1

    def linrel_callback(self):
        meas = LinrelMeasurement()
        for auv in self.auvs:
            if self.auvs[auv] == None or self.pose == None:
                continue

            # Measurement
            # meas.header.seq = self.linrelSeq
            meas.header.stamp.sec = int(self.node.get_clock().now().seconds_nanoseconds()[0])
            meas.header.stamp.nanosec = int(self.node.get_clock().now().seconds_nanoseconds()[1])
            # meas.header.frame_id = TODO proper reference frame
            meas.robot_measured = auv
            x,y,z = self.get_distance(self.pose.position, self.auvs[auv].pose.pose.position, linear=True)
            x = self.insert_noise(x, self.lin_rel_noise)
            y = self.insert_noise(y, self.lin_rel_noise)
            z = self.insert_noise(z, self.lin_rel_noise)
            # az, elev = self.get_bearing(self.pose,self.auvs[auv].pose.pose.position)
            # az = self.insert_noise(az, self.angular_noise)
            # elev = self.insert_noise(elev, self.angular_noise)
            # # rospy.loginfo("Publishing dist to " + auv)       
            # meas.range = round(_range, self.dist_res)
            # meas.azimuth = round(az, self.angular_res)
            # meas.elevation = round(elev, self.angular_res)
            # meas.fit_error = 0

            meas.x = x
            meas.y = y
            meas.z = z

            self.linrel_pub.publish(meas)
            self.linrelSeq += 1

    def insert_noise(self, meas, noise_std):
        if self.noise:
            return meas + np.random.normal(0,noise_std)
        else:
            return meas
            
    def get_distance(self, point1, point2, linear=False):
        """
        Returns the distance between 2 points
        
        Parameters
        ----------
        point1 : Point
        point2 : Point

        Returns
        -------
        float64
            Distance between points
        """
        x1, y1, z1 = point1.x, point1.y, point1.z
        x2, y2, z2 = point2.x, point2.y, point2.z
        p1_arr = np.array([x1, y1, z1])
        p2_arr = np.array([x2, y2, z2])
        diff = -p2_arr + p1_arr
        if linear:
            return diff
        else:
            return np.linalg.norm(diff)

    def get_bearing(self, pose1, point2):
        """
        Calculates the elevation and azimuth from a pose to a point in space

        Parameters
        ----------
        pose1: Pose
        point2: Point

        Returns
        -------
        azimuth: float64
            0 to 360 degrees
        elevation: float64
            -90 to 90 degrees
        """

        # Azimuth
        quat_list = [ pose1.orientation.x, \
                      pose1.orientation.y, \
                      pose1.orientation.z, \
                      pose1.orientation.w]
        (roll, pitch, yaw) = self.euler_from_quaternion(quat_list)
        azimuth = np.arctan2(point2.x - pose1.position.x, point2.y - pose1.position.y) - np.pi / 2 + yaw
        
        if azimuth < 0: # wrap negative angles
            azimuth += 2*np.pi

        # Elevation
        z_diff = point2.z - pose1.position.z
        z2_old = point2.z         # Store so we can temporarily change p2's z
        point2.z = pose1.position.z
        xy_dist = self.get_distance(point2, pose1.position)
        point2.z = z2_old
        elevation = np.arctan2(z_diff, xy_dist)

        # Convert to degrees
        azimuth = azimuth * 180 / np.pi
        elevation = elevation * 180 / np.pi
        # print(azimuth)
        # print(elevation)

        return azimuth, elevation

    def euler_from_quaternion(self,quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]

        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = np.arctan2(sinr_cosp,cosr_cosp)

        sinp = 2 * (w*y - z*x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = np.arctan2(siny_cosp,cosy_cosp)

        return [roll, pitch, yaw]

    def quaternion_from_euler(self,roll,pitch,yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        """
        cy = np.cos(yaw*0.5)
        sy = np.sin(yaw*0.5)
        cp = np.cos(pitch*0.5)
        sp = np.sin(pitch*0.5)
        cr = np.cos(roll*0.5)
        sr = np.sin(roll*0.5)

        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr
        w = cy * cp * cr + sy * sp * sr

        return [x,y,z,w]
            
def test1():
    rospy.init_node('test_sensor_node')
    # Create 2 poses and check the get bearing function

    pose1 = Pose()
    pose1.position.x = 1
    pose1.position.y = 1
    pose1.position.z = -1.43
    pose1.orientation.w = 1
    
    pose2 = Pose()
    pose2.position.z = 0
    yaw = 0
    quat_list = self.quaternion_from_euler(0,0,yaw)
    pose2.orientation.x = quat_list[0]
    pose2.orientation.y = quat_list[1]
    pose2.orientation.z = quat_list[2]
    pose2.orientation.w = quat_list[3]

    o1 = Odometry()
    o1.pose.pose = pose1
    o2 = Odometry()
    o2.pose.pose = pose2

    sp = SensorPub('rob', ['rob','bob'])
    sp.auvs['bob'] = o1
    sp.pose = o2.pose.pose
    # sp.get_distance(pose1.position, pose2.position)
    # sp.get_bearing(pose2, pose1.position)
    rospy.spin()

def main():

    cl_args = sys.argv[1:]

    # agent_cfg = load_config(get_package_share_directory('etddf_ros2')+'/ros_agent_config.yaml')
    gen_cfg = load_config(get_package_share_directory('etddf_ros2')+'/points.yaml')

    name = cl_args[0]
    active_auvs = gen_cfg[name]['meas_connections']
    active_auvs.append(name)
    
    sp = SensorPub(name, active_auvs, gen_cfg)
    rclpy.spin(sp.node)

if __name__ == "__main__":
    main()
