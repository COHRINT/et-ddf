#!/usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
import tf
import numpy as np

def rad2deg(val):
    return val * (180.0/np.pi)

def callback2(msg):
    ori = msg.pose.pose.orientation
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    print("\t"+ str([rad2deg(roll), rad2deg(pitch), rad2deg(yaw)]))

def callback(msg):
    ori = msg.pose.pose.orientation
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    print([rad2deg(roll), rad2deg(pitch), rad2deg(yaw)])

rospy.init_node("yaw_echo")
rospy.Subscriber("/bluerov2_4/pose_gt", Odometry, callback)
rospy.Subscriber("/bluerov2_4/etddf/strapdown/estimate", Odometry, callback2)
rospy.spin()