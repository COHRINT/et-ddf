#!/usr/bin/env python
from __future__ import division

import rospy
import tf
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import Imu

def callback(msg):
    ori = msg.pose.pose.orientation
    roll, pitch, yaw = tf.transformations.euler_from_quaternion( (ori.x, ori.y, ori.z, ori.w) )
    print(yaw * (180 / np.pi))
def imu_callback(imu_msg):
    # ori = msg.pose.pose.orientation
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])
    # roll, pitch, yaw = tf.transformations.euler_from_quaternion( (ori.x, ori.y, ori.z, ori.w) )
    print(yaw * (180 / np.pi))

rospy.init_node("quat2euler")

# rospy.Subscriber("bluerov2_3/odometry/filtered", Odometry, callback)
rospy.Subscriber("bluerov2_3/mavros/imu/data/corrected", Imu, imu_callback)

rospy.spin()
