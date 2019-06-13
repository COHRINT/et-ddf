#!/usr/bin/env python
from __future__ import division
"""
Simulates the movements of points in space
"""
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, TwistStamped
import random
import tf

ODOM_INDEX = 0
PUB_INDEX = 1

class PointSim:

    def __init__(self):
        rospy.loginfo("Point Sim and Controller Initialized")
        self.load_auvs()
        self.update_period = 1 / int(rospy.get_param('sim/update_freq'))
        self.timer = rospy.Timer(rospy.Duration(self.update_period), self.update_poses)

    def load_auvs(self):
        self.auvs = {} # each auv has an odom
        auv_list = rospy.get_param('/active_auvs')
        for auv in auv_list:
            start_odom = Odometry()
            start_pos = rospy.get_param(auv+'/start_pos', 'random')
            if start_pos == "random":
                start_odom.pose.pose = self.get_random_pose()
            else:
                start_odom.pose.pose = self.load_start_pose(start_pos)
            start_odom.header.seq = 0
            start_odom.header.stamp = rospy.get_rostime()
            start_odom.header.frame_id = 'world'
            start_odom.child_frame_id = auv + '/base_link'
            pub = rospy.Publisher(auv + '/pose_gt', Odometry, queue_size=10)
            self.auvs[auv] = [start_odom, pub]
            rospy.Subscriber(auv + '/new_twist', TwistStamped, self.control_callback)

    def load_start_pose(self, pose_list):
        pose = Pose()
        pose.position.x = pose_list['x']
        pose.position.y = pose_list['y']
        pose.position.z = pose_list['z']
        roll, pitch = 0,0
        yaw = pose_list['psi']
        quat_list = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quat_list[0]
        pose.orientation.y = quat_list[1]
        pose.orientation.z = quat_list[2]
        pose.orientation.w = quat_list[3]
        return pose

    def get_random_pose(self):
        [min_point, max_point] = rospy.get_param('sim/random_pose_min_max')
        size = max_point - min_point
        pose = Pose()
        pose.position.x = random.random() * size + min_point
        pose.position.y = random.random() * size + min_point
        pose.position.z = random.random() * size + min_point
        if rospy.get_param('sim/random_yaw'):
            yaw = random.random() * np.pi * 2
        else:
            yaw = 0
        roll, pitch = 0,0
        quat_list = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quat_list[0]
        pose.orientation.y = quat_list[1]
        pose.orientation.z = quat_list[2]
        pose.orientation.w = quat_list[3]
        return pose

    def update_poses(self, msg):
        for auv in self.auvs:
            odom = self.auvs[auv][ODOM_INDEX]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(( odom.pose.pose.orientation.x, \
                                                                          odom.pose.pose.orientation.y, \
                                                                          odom.pose.pose.orientation.z, \
                                                                          odom.pose.pose.orientation.w))
                                        
            roll += odom.twist.twist.angular.x * self.update_period
            pitch += odom.twist.twist.angular.y * self.update_period
            yaw += odom.twist.twist.angular.z * self.update_period
            roll = self.correct_angles(roll)
            pitch = self.correct_angles(pitch)
            yaw = self.correct_angles(yaw)

            quat_list = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            odom.pose.pose.orientation.x = quat_list[0]
            odom.pose.pose.orientation.y = quat_list[1]
            odom.pose.pose.orientation.z = quat_list[2]
            odom.pose.pose.orientation.w = quat_list[3]
            
            odom.pose.pose.position.x += odom.twist.twist.linear.x * self.update_period
            odom.pose.pose.position.y += odom.twist.twist.linear.y * self.update_period
            odom.pose.pose.position.z += odom.twist.twist.linear.z * self.update_period

            odom.header.stamp = rospy.get_rostime()

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
        
    def control_callback(self, msg):
        topic = msg._connection_header['topic']
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
        rospy.loginfo(self.auvs[auv][ODOM_INDEX])

        # Republish the new transform msg.header.frame_id -> world

def main():
    rospy.init_node('point_sim_contoller')
    ps = PointSim()
    rospy.spin()

if __name__ == "__main__":
    main()
