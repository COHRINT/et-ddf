#!/usr/bin/env python
"""

Takes in a name of a file that contains waypoints and moves the rov from one waypoint to the next.

    
"""
from __future__ import division
import rospy
from minau.srv import ArmControl, SetHeadingVelocity
from minau.msg import ControlStatus
from nav_msgs.msg import Odometry
import numpy as np
import tf
from geometry_msgs.msg import Vector3, Twist, Pose
import csv
import sys
import time
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


def normalize_angle(angle):
    return np.mod( angle + np.pi, 2*np.pi) - np.pi

def normalize_velocity(v,speed):
    """Takes in the position difference vector and returns velcity vector

    Arguments:
        v {Vector3} -- 3d vector of position difference of the rov and its waypoint
        speed {float} -- the speed we want the rov to move at

    Returns:
        Vector3 -- the velocity vector of the rov
    """
    size=np.sqrt(v.x**2+v.y**2+v.z**2)
    size = size/speed
    v.x = v.x/size
    v.y = v.y/size
    v.z = v.z/size
    return v

class Waypoint:

    def __init__(self):
        """
        Constructor the subscribes to a couple topics.
        """
        self.pose = None
        self.yaw = None
        rospy.Subscriber("pose_gt", Odometry, self.pose_callback)
        rospy.Subscriber("uuv_control/control_status", ControlStatus, self.yaw_estimate_callback)

    def pose_callback(self, msg):
        """Stores all the odometry information

        Arguments:
            msg {Odometry} -- All the position and orientation of the rov
        """
        self.pose = msg
        (r, p, self.yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    def yaw_estimate_callback(self, msg):
        self.yaw_estimate_deg = msg.current_heading



#reads in the specific csv file that holds the waypoints it is supposed to use
args = rospy.myargv(argv=sys.argv)
wayptsF = args[1]
with open(wayptsF, 'r') as read_obj:
    csv_reader = csv.reader(read_obj)
    waypts = list(csv_reader)


#this teleports the rov to the first waypoint
def set_model_state(model_name, pose):
    """Moves a model in gazebo

    Arguments:
        model_name {String} -- The name of what you want to move
        pose {Pose} -- Pose of where you want to move it to
    """
    rospy.wait_for_service('/gazebo/set_model_state')    
    for i in range(3): # repeat 3 times, sometimes gazebo doesn't actually move the model but indicates it does in its modelstate...    
        result = None
        try:
            mover = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            res = mover(ModelState(model_name, pose, Twist(), "world") )
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        time.sleep(0.1)

initial_point = Pose()
initial_point.position.x = float(waypts[0][0])
initial_point.position.y = float(waypts[0][1])
initial_point.position.z = float(waypts[0][2])
nameS = rospy.get_namespace()
set_model_state(nameS[1:-1],initial_point)

rospy.init_node("waypoint_mover")
# arm a single bluerov
# rospy.loginfo("starting up")
rospy.wait_for_service('uuv_control/arm_control')
try:
    arm_control = rospy.ServiceProxy('uuv_control/arm_control', ArmControl)
    resp1 = arm_control()
    # print(resp1)
except rospy.ServiceException, e:
    print "Arm Control Service call failed: %s"%e

wp = Waypoint()

#start with second waypoint
waypt_num = 1
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    if wp.yaw != None:
        target_waypt = waypts[waypt_num]

        # Check 
        # in position first
        # print(target_waypt)
        diff_x = float(target_waypt[0]) - wp.pose.pose.pose.position.x
        diff_y = float(target_waypt[1]) - wp.pose.pose.pose.position.y
        diff_z = -float(target_waypt[2]) + wp.pose.pose.pose.position.z
        dist = np.linalg.norm([diff_x, diff_y,diff_z])
        if dist < 1:
            waypt_num = (waypt_num + 1) % len(waypts)

        # Get New control
        target_waypt = waypts[waypt_num]
        diff_x = float(target_waypt[0]) - wp.pose.pose.pose.position.x
        diff_y = float(target_waypt[1]) - wp.pose.pose.pose.position.y
        diff_z = -float(target_waypt[2]) + wp.pose.pose.pose.position.z
        print("target_waypt: " + str(target_waypt))
        print("x_diff: " + str(diff_x))
        print("y_diff: " + str(diff_y))
        print("z_diff: " + str(diff_z))

        ang = np.arctan2(diff_x, diff_y) # NED
        v3 = Vector3(diff_y,diff_x,diff_z)
        v3 = normalize_velocity(v3,0.5)


        # Set Heading Velocity
        rospy.wait_for_service('uuv_control/set_heading_velocity')
        try:
            shv = rospy.ServiceProxy('uuv_control/set_heading_velocity', SetHeadingVelocity)
            resp1 = shv(ang * (180/np.pi), v3)
        except rospy.ServiceException, e:
            print "Set Heading Velocity Service call failed: %s"%e

    rate.sleep()
