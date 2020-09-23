#!/usr/bin/env python
from __future__ import division

import rospy
import tf
from nav_msgs.msg import Odometry
import numpy as np

def callback(msg):
    ori = msg.pose.pose.orientation
    roll, pitch, yaw = tf.transformations.euler_from_quaternion( (ori.x, ori.y, ori.z, ori.w) )
    print(yaw * (180 / np.pi))

rospy.init_node("quat2euler")

rospy.Subscriber("/bluerov2_4/odometry/filtered", Odometry, callback)

rospy.spin()
