#!/usr/bin/env python
from __future__ import division
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Vector3



class DVLSensor:

    def __init__(self):
        self.pose = None
        self.yaw = None
        rospy.Subscriber("pose_gt", Odometry, self.pose_callback)
    def pose_callback(self, msg):
        self.pose = msg
        self.vel = msg.twist.twist.linear



rospy.init_node('dvl_sensor')
pub = rospy.Publisher('dvl',Vector3,queue_size=10)

ds = DVLSensor()

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    if ds.pose != None:
        #these add guasian noise to each of the coordinates, the first argument is the mean and the second is the standard deviation
        dx = np.random.normal(0,.1)
        dy = np.random.normal(0,.1)
        dz = np.random.normal(0,.1)
        v = Vector3(ds.vel.x+dx,ds.vel.y+dy,ds.vel.z+dz)
        pub.publish(v)
    rate.sleep()