#!/usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from ping360_sonar.msg import SonarEcho
import numpy as np
import tf

rospy.init_node("visualize_scan_beam")
pose = None
pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

SEQ_START = 7
seq = SEQ_START

last_marker = m = Marker()
m.header.frame_id = "map"
m.id = seq
m.type = m.ARROW
m.ns = ""

def scan_callback(msg):
    global pose, pub, seq, last_marker
    if pose is not None:
        angle = msg.angle * (np.pi / 200.0) # Convert grad to rad
        angle += np.pi # Transform to baselink frame

        ori = pose.orientation
        _, _, robot_yaw = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        quat = tf.transformations.quaternion_from_euler(0, 0, angle + robot_yaw)

        # Delete last marker
        m = last_marker
        m.action = m.DELETE
        m.pose.position = pose.position
        m.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        m.scale = Vector3(1,0.1,1)
        m.color = ColorRGBA(2,0,0,1)
        pub.publish(m)
        last_marker = m

        # Publish new marker
        m = Marker()
        m.header.frame_id = "map"
        m.id = seq
        m.type = m.ARROW
        m.ns = ""
        m.pose.position = pose.position
        m.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        m.scale = Vector3(1,0.1,1)
        m.color = ColorRGBA(2,0,0,1)
        pub.publish(m)
        last_marker = m

def pose_callback(msg):
    global pose
    pose = msg.pose.pose

pose_topic = "etddf/estimate" + rospy.get_namespace()[:-1]
rospy.Subscriber(pose_topic, Odometry, pose_callback)
rospy.wait_for_message(pose_topic, Odometry)
rospy.Subscriber("ping360_node/sonar/data", SonarEcho, scan_callback)
rospy.spin()