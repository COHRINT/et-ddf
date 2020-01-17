#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from etddf_ros.msg import gpsMeasurement, linrelMeasurement, depthMeasurement, usblMeasurement
from geometry_msgs.msg import Pose
from decimal import Decimal
import numpy as np
import tf
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

    vehicle_names = {}
    
    def __init__(self, name, id, active_robots):

        # create subcribers to ground truth topics for vehicles of interest
        self.name = name
        gt_topic = rospy.get_param('gt_topic')
        for rob in active_robots:
            rospy.Subscriber('/' + rob + '/' + gt_topic, Odometry, self.pose_callback)
            self.vehicle_names[rob] = None
        self.pose = None

        # Publishers
        self.depth_pub = rospy.Publisher('depth', depthMeasurement, queue_size=10)
        self.usbl_pub = rospy.Publisher('usbl', usblMeasurement, queue_size=10)
        self.gps_pub = rospy.Publisher('gps',gpsMeasurement,queue_size=10)
        self.linrel_pub = rospy.Publisher('lin_rel',linrelMeasurement,queue_size=10)

        # Timer callbacks
        depth_rate = 1 / float(rospy.get_param('depth_pub_rate', 50))
        usbl_rate = 1 / float(rospy.get_param('usbl_pub_rate', 0.5))
        
        self.depth_timer = rospy.Timer(rospy.Duration(depth_rate), self.depth_callback)
        self.usbl_timer = rospy.Timer(rospy.Duration(usbl_rate), self.usbl_callback)
        
        gps_rate = 1 / float(rospy.get_param('gps_pub_rate', 10))
        linrel_rate = 1 / float(rospy.get_param('lin_rel_pub_rate', 0.5))
        
        self.gps_timer = rospy.Timer(rospy.Duration(gps_rate), self.gps_callback)
        self.linrel_timer = rospy.Timer(rospy.Duration(linrel_rate), self.linrel_callback)

        # Noise
        self.noise = rospy.get_param('noise',False)
        self.depth_noise = float(rospy.get_param('depth_noise', 0.03))
        self.angular_noise = float(rospy.get_param('angular_noise',1))
        self.distance_noise = float(rospy.get_param('distance_noise', 1.0))
        self.gps_noise = float(rospy.get_param('gps_noise'))
        self.lin_rel_noise = float(rospy.get_param('lin_rel_noise'))

        # Resolutions
        self.dist_res = int(rospy.get_param('distance_res', 1))
        self.angular_res = int(rospy.get_param('angular_res', 1))
        self.depth_res = int(rospy.get_param('depth_res', 2))
        self.pos_res = int(rospy.get_param('positioning_res',1))
        
        rospy.loginfo("Sensor pub for " + name + " initialized")
        self.gpsSeq = 0
        self.linrelSeq = 0
        self.depthSeq = 0
        self.usblSeq = 0

    def pose_callback(self, msg):
        topic = msg._connection_header['topic']
        vehicle = None
        for name in self.vehicle_names.keys():
            if name in topic:
                vehicle = name
                break
        if vehicle == self.name:
            self.pose = msg.pose.pose
        else:
            self.vehicle_names[vehicle] = msg

    def depth_callback(self, msg):
        if self.pose == None:
            return
        depth = self.pose.position.z
        depth = self.insert_noise(depth, self.depth_noise)
        meas = depthMeasurement()
        meas.data = round(depth, self.depth_res)
        meas.header.seq = self.depthSeq
        meas.header.stamp = rospy.Time.now()
        # meas.header.frame_id = TODO proper reference frame
        self.depth_pub.publish(meas)
        self.depthSeq += 1
        
    def usbl_callback(self, msg):
        meas = usblMeasurement()
        for name in self.vehicle_names:
            if self.vehicle_names[name] == None or self.pose == None:
                continue

            # Measurement
            meas.header.seq = self.usblSeq
            meas.header.stamp = rospy.Time.now()
            # meas.header.frame_id = TODO proper reference frame
            meas.robot_measured = name
            _range = self.get_distance(self.pose.position, self.vehicle_names[name].pose.pose.position)
            _range = self.insert_noise(_range, self.distance_noise)
            az, elev = self.get_bearing(self.pose,self.vehicle_names[name].pose.pose.position)
            az = self.insert_noise(az, self.angular_noise)
            elev = self.insert_noise(elev, self.angular_noise)
            # rospy.loginfo("Publishing dist to " + name)
            meas.range = round(_range, self.dist_res)
            meas.azimuth = round(az, self.angular_res)
            meas.elevation = round(elev, self.angular_res)
            meas.fit_error = 0
            self.usbl_pub.publish(meas)
            self.usblSeq += 1

    def gps_callback(self,msg):
        if self.pose == None:
            return
        x = self.pose.position.x
        y = self.pose.position.y
        z = self.pose.position.z
        x = self.insert_noise(x, self.gps_noise)
        y = self.insert_noise(y, self.gps_noise)
        z = self.insert_noise(z, self.gps_noise)
        meas = gpsMeasurement()
        # meas.data = round(depth, self.depth_res)

        meas.x = x
        meas.y = y
        meas.z = z

        meas.header.seq = self.gpsSeq
        meas.header.stamp = rospy.Time.now()
        # meas.header.frame_id = TODO proper reference frame
        self.gps_pub.publish(meas)
        self.gpsSeq += 1

    def linrel_callback(self,msg):
        # rospy.loginfo("Publishing measurement")
        meas = linrelMeasurement()
        for name in self.vehicle_names:
            if self.vehicle_names[name] == None or self.pose == None:
                continue

            # Measurement
            meas.header.seq = self.linrelSeq
            meas.header.stamp = rospy.Time.now()
            # meas.header.frame_id = TODO proper reference frame
            meas.robot_measured = name
            x,y,z = self.get_distance(self.pose.position, self.vehicle_names[name].pose.pose.position, linear=True)
            x = self.insert_noise(x, self.lin_rel_noise)
            y = self.insert_noise(y, self.lin_rel_noise)
            z = self.insert_noise(z, self.lin_rel_noise)
            # az, elev = self.get_bearing(self.pose,self.vehicle_names[name].pose.pose.position)
            # az = self.insert_noise(az, self.angular_noise)
            # elev = self.insert_noise(elev, self.angular_noise)
            # # rospy.loginfo("Publishing dist to " + name)
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
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quat_list)
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
    quat_list = tf.transformations.quaternion_from_euler(0,0,yaw)
    pose2.orientation.x = quat_list[0]
    pose2.orientation.y = quat_list[1]
    pose2.orientation.z = quat_list[2]
    pose2.orientation.w = quat_list[3]

    o1 = Odometry()
    o1.pose.pose = pose1
    o2 = Odometry()
    o2.pose.pose = pose2

    sp = SensorPub('rob', 0, ['rob','bob'])
    sp.vehicle_names['bob'] = o1
    sp.pose = o2.pose.pose
    # sp.get_distance(pose1.position, pose2.position)
    # sp.get_bearing(pose2, pose1.position)
    rospy.spin()

def main():
    rospy.init_node('sensor_pub', anonymous=True)

    # get vehicle name and id from namespace
    name = rospy.get_namespace().split('/')[1]
    agent_id = int(name.split('_')[1])

    # get id location in all ordered ids
    connections = rospy.get_param('connections')
    ordered_ids = []
    for conn in connections:
        ordered_ids += conn
    ordered_ids = sorted(list(set(ordered_ids))) # remove non unique values

    # collect names of robots this robot is connected to, and add ownship name
    connected_vehicle_names = []
    active_vehicles = rospy.get_param('active_vehicles')
    for conn in connections[ordered_ids.index(agent_id)]:
        for vehicle_name in active_vehicles:
            if int(vehicle_name.split('_')[1]) == conn:
                connected_vehicle_names.append(vehicle_name)
    connected_vehicle_names.append(name)
    
    sp = SensorPub(name, agent_id, connected_vehicle_names)
    rospy.spin()

if __name__ == "__main__":
    try:
        # test1()
        main()
    except rospy.ROSInterruptException:
        pass
