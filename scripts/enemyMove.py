#!/usr/bin/env python
from __future__ import division
import rospy
from nav_msgs.msg import Odometry
from uuv_control_msgs.msg import WaypointSet, Waypoint
from uuv_control_msgs.srv import GoTo
import numpy as np
import tf
from geometry_msgs.msg import Vector3, Point
import random

#check distance between waypoint and location
def dist(E,W):
    x = E.pose.pose.pose.position.x - W.point.x
    y = E.pose.pose.pose.position.y - W.point.y
    z = E.pose.pose.pose.position.z - W.point.z
    return np.linalg.norm([x,y,z])

class EnemyMove:
    def __init__(self):
        self.pose = None
        self.yaw = None
        rospy.Subscriber("pose_gt", Odometry, self.pose_callback)

    def pose_callback(self, msg):
        self.pose = msg
        (r, p, self.yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

rospy.init_node("track_rov")
print('Hello')
#rospy.wait_for_service('dp_controller/waypoints')
print('Goodbye')

#make the waypoint
w = Waypoint()
p = Point(1,2,-4)
w.point = p
w.max_forward_speed = 0.4
w.heading_offset = 0
w.use_fixed_heading = False
#initiate the enemy rover
em = EnemyMove()
rate = rospy.Rate(1)



while not rospy.is_shutdown():
    if em.yaw != None:
        #determine random location
        x = (random.random()*10) -5
        y = (random.random()*10) -5
        z = -random.random()*5 -2.5
        P1 = Point(x,y,z)
        w.point = P1
        try:
            rospy.wait_for_service('go_to',timeout=30)
            print('hello')
        except rospy.ROSException():
            print('failed')
        gt = rospy.ServiceProxy('go_to',GoTo)
        res1 = gt(w,0.4,'lipb')
        #wait to determine a new waypoint until distance is really small
        while dist(em,w) > 0.05:
            a = 1






