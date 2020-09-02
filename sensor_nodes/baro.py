#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np

rospy.init_node("baro")
pub = rospy.Publisher("baro", Float64, queue_size=2)

rate = rospy.Rate(20)

def callback(msg):
    global pub
    x = msg.pose.pose.position.z + np.random.normal(0, scale=0.01)
    new_msg = Float64(x)
    pub.publish(new_msg)
    # rate.sleep()

rospy.Subscriber("pose_gt", Odometry, callback, queue_size=1)
rospy.spin()