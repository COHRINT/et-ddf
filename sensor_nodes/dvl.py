#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import numpy as np

rospy.init_node("dvl")
pub = rospy.Publisher("dvl", Vector3, queue_size=2)

rate = rospy.Rate(10)

def callback(msg):
    global pub
    dvl_x = msg.twist.twist.linear.x + np.random.normal(0, scale=0.005)
    dvl_y = msg.twist.twist.linear.y + np.random.normal(0, scale=0.005)
    dvl_z = msg.twist.twist.linear.z + np.random.normal(0, scale=0.005)
    new_msg = Vector3(dvl_x, dvl_y, dvl_z)
    pub.publish(new_msg)
    rate.sleep()

rospy.Subscriber("pose_gt", Odometry, callback, queue_size=1)
rospy.spin()