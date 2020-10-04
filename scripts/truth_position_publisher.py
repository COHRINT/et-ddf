#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

truth_position = Point(1.22, -1.22, -0.5)

rospy.init_node("truth_publisher")
pub = rospy.Publisher("pose_gt", Odometry, queue_size=10)
pub3 = rospy.Publisher("/bluerov2_3/pose_gt", Odometry, queue_size=10)

def callback(msg):
    global truth_position, pub, pub3
    msg.pose.pose.position = truth_position
    pub.publish(msg)

    # Other bluerov2 publisher
    msg3 = Odometry()
    msg3.header = msg.header
    msg3.pose.pose.position = Point(1.22, 1.22, -0.45)
    msg3.pose.pose.orientation.w = 1
    pub3.publish(msg3)


rospy.Subscriber("odometry/filtered", Odometry, callback)
rospy.spin()
