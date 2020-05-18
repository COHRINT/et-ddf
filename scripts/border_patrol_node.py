#!/usr/bin/env python
from __future__ import division
import rospy
from minau.srv import ArmControl, SetHeadingVelocity
from minau.msg import ControlStatus
from nav_msgs.msg import Odometry
import numpy as np
import tf
from geometry_msgs.msg import Vector3

def normalize_angle(angle):
    return np.mod( angle + np.pi, 2*np.pi) - np.pi

class BorderPatrol:

    def __init__(self):
        self.pose = None
        self.yaw = None
        rospy.Subscriber("pose_gt", Odometry, self.pose_callback)
        rospy.Subscriber("uuv_control/control_status", ControlStatus, self.yaw_estimate_callback)

    def pose_callback(self, msg):
        self.pose = msg
        (r, p, self.yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    def yaw_estimate_callback(self, msg):
        self.yaw_estimate_deg = msg.current_heading
        

# waypts = [[10,10],[10,-10],[-10,-10],[-10,10]]
waypts = [[5,5],[5,-5],[-5,-5],[-5,5]]

rospy.init_node("border_patrol")
# arm a single bluerov
rospy.loginfo("starting up")
rospy.wait_for_service('uuv_control/arm_control')
try:
    arm_control = rospy.ServiceProxy('uuv_control/arm_control', ArmControl)
    resp1 = arm_control()
    print(resp1)
except rospy.ServiceException, e:
    print "Arm Control Service call failed: %s"%e

bp = BorderPatrol()

waypt_index = rospy.get_param("~waypt_i", 0)
rospy.loginfo("waypt_index: " + str(waypt_index))

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    if bp.yaw != None:
        z_goal = 1.0
        target_waypt = waypts[waypt_index]

        # Check in position first
        diff_x = target_waypt[0] - bp.pose.pose.pose.position.x
        diff_y = target_waypt[1] - bp.pose.pose.pose.position.y
        dist = np.linalg.norm([diff_x, diff_y])
        if dist < 1:
            waypt_index = (waypt_index + 1) % len(waypts)

        # Get New control
        target_waypt = waypts[waypt_index]
        print("target_waypt: " + str(target_waypt))
        diff_x = target_waypt[0] - bp.pose.pose.pose.position.x
        diff_y = target_waypt[1] - bp.pose.pose.pose.position.y
        diff_z = z_goal + bp.pose.pose.pose.position.z
        print("x_diff: " + str(diff_x))
        print("y_diff: " + str(diff_y))

        ang = np.arctan2(diff_x, diff_y) # NED

        # Set heading directly
        # ang_err = normalize_angle(ang - bp.yaw)
        # ang_vel = ang_err if np.abs(ang_err) < np.pi/10 else (np.pi/10) * np.sign(ang_err)
        dist_err = np.linalg.norm([diff_x, diff_y])
        ang_est_error = bp.yaw*(180/np.pi) - bp.yaw_estimate_deg
        x_dot = 0.4 * np.sign(diff_y) if np.abs(diff_y) > 0.8 else 0.01 * np.sign(diff_y)
        y_dot = 0.4 * np.sign(diff_x) if np.abs(diff_x) > 0.8 else 0.01 * np.sign(diff_x)

        z_dot = 0.1 * np.sign(diff_z) if np.abs(diff_z) > 0.5 else 0.01 * np.sign(diff_z)

        # Set Heading Velocity
        rospy.wait_for_service('uuv_control/set_heading_velocity')
        try:
            shv = rospy.ServiceProxy('uuv_control/set_heading_velocity', SetHeadingVelocity)
            v3 = Vector3(x_dot, y_dot, z_dot)
            resp1 = shv(ang * (180/np.pi), v3)
        except rospy.ServiceException, e:
            print "Set Heading Velocity Service call failed: %s"%e

    rate.sleep()
