#!/usr/bin/env python
from __future__ import division
import rospy
import random
from geometry_msgs.msg import TwistStamped
import numpy as np

class Planner:

    def __init__(self, name, update_period):
        self.name = name
        self.pub = rospy.Publisher('new_twist', TwistStamped, queue_size=10)
        self.planner = rospy.get_param('planner')
        start_twist = rospy.get_param('start_twist')
        if start_twist == 'random':
            self.twist = self.get_random_twist()
        else:
            self.twist = self.load_twist_dict(start_twist)
        rospy.Timer(rospy.Duration(update_period), self.pub_cmd)
        rospy.loginfo(name + " Planner Initialized.")
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
        [min_twist, max_twist] = rospy.get_param('/planners/random_linear_vel_range')
        [min_ang, max_ang] = rospy.get_param('/planners/random_angular_vel_range')
        size = max_twist - min_twist
        twist = TwistStamped()
        twist.twist.linear.x = random.random() * size + min_twist
        twist.twist.linear.y = random.random() * size + min_twist
        # twist.twist.linear.z = random.random() * size + min_twist
        size_ang = max_ang - min_ang
        twist.twist.angular.z = random.random() * size_ang + min_ang
        return twist
        
    def pub_cmd(self, msg):
        new_twist = self.get_new_twist()
        if new_twist == None: # publish the old velocity
            new_twist = self.twist
        else:
            self.twist = new_twist
        new_twist.header.seq = self.seq
        new_twist.header.stamp = rospy.Time.now()
        new_twist.header.frame_id = self.name + "/base_link"
        self.pub.publish(new_twist)
        self.seq += 1

    def get_new_twist(self):
        """ This function provides an easy place to add more complex planners  that actively change the velocity """
        if self.planner == "linear":
            return None

def main():
    rospy.init_node('point_planner', anonymous=True)
    name = rospy.get_namespace().split('/')[1]
    while not rospy.has_param('/planners') and not rospy.is_shutdown(): # wait for the param server to load
        pass
    param_name = rospy.search_param('planners/update_freq')
    update_period = 1 / int(rospy.get_param(param_name))
    p = Planner(name, update_period)
    rospy.spin()
    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
