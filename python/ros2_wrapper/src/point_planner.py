#!/usr/bin/env python

from __future__ import division

import sys
import rclpy
import random
from geometry_msgs.msg import TwistStamped
import numpy as np

from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory
from etddf.helpers.config_handling import load_config

class Planner:

    def __init__(self, name, update_period, cfg):

        rclpy.init()
        self.node = rclpy.create_node('planner')

        self.name = name
        self.pub = self.node.create_publisher(TwistStamped, 'new_twist')
        self.planner = cfg[name]['planner']
        start_twist = cfg[name]['start_twist']
        self.rand_lin_vel_range = cfg['planners']['random_linear_vel_range']
        self.rand_ang_vel_range = cfg['planners']['random_angular_vel_range']

        if start_twist == 'random':
            self.twist = self.get_random_twist()
        else:
            self.twist = self.load_twist_dict(start_twist)
        self.timer = self.node.create_timer(update_period, self.pub_cmd)
        self.node.get_logger().info(name + " Planner Initialized.")
        self.seq = 0
        
    def load_twist_dict(self, twist_dict):
        dot_x = twist_dict['x']
        dot_y = twist_dict['y']
        dot_z = twist_dict['z']
        dot_psi = twist_dict['psi']
        twist = TwistStamped()
        twist.twist.linear.x = dot_x
        twist.twist.linear.y = dot_y
        twist.twist.linear.z = dot_z
        twist.twist.angular.z = dot_psi
        return twist

    
    def get_random_twist(self):
        [min_twist, max_twist] = self.rand_lin_vel_range
        [min_ang, max_ang] = self.rand_ang_vel_range
        size = max_twist - min_twist
        twist = TwistStamped()
        twist.twist.linear.x = random.random() * size + min_twist
        twist.twist.linear.y = random.random() * size + min_twist
        # twist.twist.linear.z = random.random() * size + min_twist
        size_ang = max_ang - min_ang
        twist.twist.angular.z = random.random() * size_ang + min_ang
        return twist
        
    def pub_cmd(self):
        new_twist = self.get_new_twist()
        if new_twist == None: # publish the old velocity
            new_twist = self.twist
        else:
            self.twist = new_twist
        # new_twist.header.seq = self.seq
        # print(type(self.node.get_clock().now().seconds_nanoseconds()[1]))
        new_twist.header.stamp.sec = int(self.node.get_clock().now().seconds_nanoseconds()[0])
        new_twist.header.stamp.nanosec = int(self.node.get_clock().now().seconds_nanoseconds()[1])
        new_twist.header.frame_id = self.name + "/base_link"
        self.pub.publish(new_twist)
        self.seq += 1

    def get_new_twist(self):
        """ This function provides an easy place to add more complex planners  that actively change the velocity """
        if self.planner == "linear":
            new_twist = TwistStamped()
            new_twist.twist.linear.x = self.twist.twist.linear.x + 0.1*np.cos(0.5*self.seq)
            new_twist.twist.linear.y = self.twist.twist.linear.y + 0.1*np.sin(0.5*self.seq)
            return new_twist

def main():

    # get launch arguments
    cl_args = sys.argv[1:]

    # load config
    gen_cfg = load_config(get_package_share_directory('etddf_ros2')+'/points.yaml')

    name = cl_args[0]
    update_period = 1 / int(gen_cfg['planners']['update_freq'])

    p = Planner(name, update_period, gen_cfg)
    rclpy.spin(p.node)
    
if __name__ == "__main__":
    main()
