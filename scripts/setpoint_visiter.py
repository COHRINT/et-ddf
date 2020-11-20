#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import os
import numpy as np

pose = None

def callback(msg):
    global pose
    pose = msg.pose.pose


rospy.init_node("setpoint_visiter")
rospy.Subscriber("strapdown/estimate", Odometry, callback)
rospy.wait_for_message("strapdown/estimate", Odometry)
# pose_topic = "etddf/estimate" + rospy.get_namespace()[:-1]
# rospy.Subscriber(pose_topic, Odometry, callback)
# rospy.wait_for_message(pose_topic, Odometry)


os.system("rosservice call " + rospy.get_namespace() + "uuv_control/arm_control {}")
print("armed...")

current_index = 0
setpoint = list(np.random.uniform(-20,20, 2))

heading_setpoint = 0.0
depth_setpoint = 1.0


os.system("rosservice call " + rospy.get_namespace() + "uuv_control/set_heading_depth 'heading: " + str(heading_setpoint) + "\ndepth: "+str(depth_setpoint) + "'") 

print("diving...")
rospy.sleep(5)

r = rospy.Rate(0.5)

while not rospy.is_shutdown():
    x_target = setpoint[0]
    y_target = setpoint[1]
    x_current = pose.position.x
    y_current = pose.position.y
    diff_x = x_target - x_current
    diff_y = y_target - y_current
    ori = np.arctan2(diff_y, diff_x)

    if np.abs(diff_x) < 1 and np.abs(diff_y) < 1:
        setpoint = list(np.random.uniform(-20,20, 2))
        print("############ NEXT WAYPOINT ###################")
        continue

    x_vel = 0.3 * np.cos(ori)
    y_vel = 0.3 * np.sin(ori)
    print([y_vel, x_vel])

    # transform ENU --> NED
    x_vel, y_vel = y_vel, x_vel
    z_vel = 0.0
    heading_setpoint = 90 - (ori * (180/np.pi))
    os.system("rosservice call " + rospy.get_namespace() + "uuv_control/set_heading_velocity '{heading: " + str(heading_setpoint) + ", velocity: {x: "+str(x_vel) + ", y: " + str(y_vel) + ", z: " + str(z_vel) + "}}'") 
    # rospy.sleep(10)
    r.sleep()