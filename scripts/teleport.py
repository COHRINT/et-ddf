#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
import math
import tf
import time
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from minau.srv import SetHeadingVelocity
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


#* /bluerov2_4/set_pose [geometry_msgs/PoseWithCovarianceStamped] 1 subscriber
#/rexrov2/pose_gt/change_state


rospy.init_node("Teleport")


def set_model_state(model_name, pose):
    service = '/'+model_name+'/uuv_control/set_heading_velocity'
    rospy.wait_for_service(service)
    print('got service1')
    try:
        shv = rospy.ServiceProxy(service, SetHeadingVelocity)
        v3 = Vector3(0, 0, 0)
        resp1 = shv(0, v3)
        print('set velocity')
    except rospy.ServiceException, e:
        print "Set Heading Velocity Service call failed: %s"%e
    rospy.wait_for_service('/gazebo/set_model_state')    
    for i in range(3): # repeat 3 times, sometimes gazebo doesn't actually move the model but indicates it does in its modelstate...    
        result = None
        try:
            mover = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            res = mover( ModelState(model_name, pose, Twist(), "world") )
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        time.sleep(0.1)

p1 = Pose()
p1.position.x = 5
p1.position.y = -5
p1.position.z = -2.5

p2 = Pose()
p2.position.y = 0
p2.position.x = -0.5
p2.position.z = 0

p3 = Pose()
p3.position.y = 0
p3.position.x = 0.5
p3.position.z = 0

set_model_state('bluerov2_3',p2)
# set_model_state('bluerov2_4',p3)