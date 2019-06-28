#!/usr/bin/env python
from __future__ import division
"""
Simulates the movements of points in space
"""

import sys
import rclpy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, TwistStamped
import random
# import tf
from functools import partial

from etddf.helpers.config_handling import load_config
from ament_index_python.packages import get_package_share_directory

ODOM_INDEX = 0
PUB_INDEX = 1

class PointSim:

    def __init__(self,cfg):

        # initialize node
        rclpy.init()
        self.node = rclpy.create_node('point_sim_controller')

        self.node.get_logger().info("Point Sim and Controller Initialized")
        self.load_auvs(cfg)
        self.update_period = 1 / int(cfg['sim']['update_freq'])
        self.timer = self.node.create_timer(self.update_period, self.update_poses)

    def load_auvs(self,cfg):
        self.auvs = {} # each auv has an odom
        auv_list = cfg['active_auvs']
        for auv in auv_list:
            start_odom = Odometry()
            start_pos = cfg[auv]['start_pos']
            if start_pos == "random":
                start_odom.pose.pose = self.get_random_pose(cfg)
            else:
                start_odom.pose.pose = self.load_start_pose(start_pos)
            # start_odom.header.seq = 0
            start_odom.header.stamp.sec = int(self.node.get_clock().now().seconds_nanoseconds()[0])
            start_odom.header.stamp.nanosec = int(self.node.get_clock().now().seconds_nanoseconds()[1])
            start_odom.header.frame_id = 'world'
            start_odom.child_frame_id = auv + '/base_link'
            self.pub = self.node.create_publisher(Odometry, auv + '/pose_gt')
            self.auvs[auv] = [start_odom, self.pub]
            self.node.create_subscription(TwistStamped, auv + '/new_twist', partial(self.control_callback, auv))

    def load_start_pose(self, pose_list):
        pose = Pose()
        pose.position.x = pose_list['x']
        pose.position.y = pose_list['y']
        pose.position.z = pose_list['z']
        roll, pitch = 0,0
        yaw = pose_list['psi']
        quat_list = self.quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quat_list[0]
        pose.orientation.y = quat_list[1]
        pose.orientation.z = quat_list[2]
        pose.orientation.w = quat_list[3]
        return pose

    def get_random_pose(self,cfg):
        [min_point, max_point] = cfg['sim']['random_pose_min_max']
        size = max_point - min_point
        pose = Pose()
        pose.position.x = random.random() * size + min_point
        pose.position.y = random.random() * size + min_point
        pose.position.z = random.random() * size + min_point
        if cfg['sim']['random_yaw']:
            yaw = random.random() * np.pi * 2
        else:
            yaw = 0
        roll, pitch = 0,0
        quat_list = self.quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quat_list[0]
        pose.orientation.y = quat_list[1]
        pose.orientation.z = quat_list[2]
        pose.orientation.w = quat_list[3]
        return pose

    def update_poses(self):
        for auv in self.auvs:
            odom = self.auvs[auv][ODOM_INDEX]
            roll, pitch, yaw = self.euler_from_quaternion(( odom.pose.pose.orientation.x, \
                                                                          odom.pose.pose.orientation.y, \
                                                                          odom.pose.pose.orientation.z, \
                                                                          odom.pose.pose.orientation.w))
                                        
            roll += odom.twist.twist.angular.x * self.update_period
            pitch += odom.twist.twist.angular.y * self.update_period
            yaw += odom.twist.twist.angular.z * self.update_period
            roll = self.correct_angles(roll)
            pitch = self.correct_angles(pitch)
            yaw = self.correct_angles(yaw)

            quat_list = self.quaternion_from_euler(roll, pitch, yaw)
            odom.pose.pose.orientation.x = quat_list[0]
            odom.pose.pose.orientation.y = quat_list[1]
            odom.pose.pose.orientation.z = quat_list[2]
            odom.pose.pose.orientation.w = quat_list[3]
            
            odom.pose.pose.position.x += odom.twist.twist.linear.x * self.update_period
            odom.pose.pose.position.y += odom.twist.twist.linear.y * self.update_period
            odom.pose.pose.position.z += odom.twist.twist.linear.z * self.update_period

            odom.header.stamp.sec = int(self.node.get_clock().now().seconds_nanoseconds()[0])
            odom.header.stamp.nanosec = int(self.node.get_clock().now().seconds_nanoseconds()[1])

            self.auvs[auv][ODOM_INDEX] = odom
            self.auvs[auv][PUB_INDEX].publish(odom)

    def correct_angles(self, angle):
        """ Map all angles between -pi to pi """
        while angle < -np.pi or angle > np.pi:
            if angle < -np.pi:
                angle += 2 * np.pi
            else:
                angle -= 2 * np.pi
        return angle
        
    def control_callback(self, name, msg):
        # topic = msg._connection_header['topic']
        topic = name
        auv = None
        for auv_name in self.auvs:
            if auv_name in topic:
                auv = auv_name
                break

        """ According to nav_msgs.msg/Odometry standards, the pose is given in header/frame_id and the twist is given in child_frame_id
        """
        self.auvs[auv][ODOM_INDEX].child_frame_id = msg.header.frame_id
        self.auvs[auv][ODOM_INDEX].header = msg.header        
        self.auvs[auv][ODOM_INDEX].header.frame_id = 'world'
        self.auvs[auv][ODOM_INDEX].twist.twist.linear = msg.twist.linear
        self.auvs[auv][ODOM_INDEX].twist.twist.angular = msg.twist.angular
        # self.node.get_logger().info(self.auvs[auv][ODOM_INDEX])

        # Republish the new transform msg.header.frame_id -> world

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

def main():

    try:
        # get cl args
        # cl_args = sys.argv[1:]

        # load config
        gen_cfg = load_config(get_package_share_directory('etddf_ros2')+'/points.yaml')

        ps = PointSim(gen_cfg)
        rclpy.spin(ps.node)
    except KeyboardInterrupt as e:
        print(e)
        ps.node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
