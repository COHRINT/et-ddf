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


os.system("rosservice call /bluerov2_4/uuv_control/arm_control {}")
print("armed...")

current_index = 0
setpoints = [[8, -3.2, 1.0], [13.5, -7.0, 1.0], [13.5, 4.0, 1.0]]

heading_setpoint = 0.0
depth_setpoint = 1.0


os.system("rosservice call /bluerov2_4/uuv_control/set_heading_depth 'heading: " + str(heading_setpoint) + "\ndepth: "+str(depth_setpoint) + "'") 

print("diving...")
rospy.sleep(20)

r = rospy.Rate(1)

while not rospy.is_shutdown():
    x_target = setpoints[current_index][0]
    y_target = setpoints[current_index][1]
    x_current = pose.position.x
    y_current = pose.position.y
    diff_x = x_target - x_current
    diff_y = y_target - y_current
    ori = np.arctan2(diff_y, diff_x)

    if np.abs(diff_x) < 1 and np.abs(diff_y) < 1:
        current_index += 1
        continue

    x_vel = 0.1*np.cos(ori)
    y_vel = 0.1*np.sin(ori)

    # transform ENU --> NED
    x_vel, y_vel = y_vel, x_vel
    z_vel = 0.0
    heading_setpoint = 90 - (ori * (180/np.pi))
    os.system("rosservice call /bluerov2_4/uuv_control/set_heading_velocity '{heading: " + str(heading_setpoint) + ", velocity: {x: "+str(y_vel) + ", y: " + str(x_vel) + ", z: " + str(z_vel) + "}}'") 
    # rospy.sleep(10)
    r.sleep()