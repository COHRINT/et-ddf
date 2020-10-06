#!/usr/bin/env python


"""
To play a hardware scenario back in RVIZ
(Run the topic extractor)
Launch RVIZ
roslaunch etddf marker_visualization.launch __ns:=bluerov2_4
roslaunch ping360_sonar detection.launch __ns:=bluerov2_4
rosrun etddf truth_position_publisher.py __ns:=bluerov2_4 # make sure it has the right truth position(s)
roslaunch etddf etddf_sonar_associate.launch __ns:=bluerov2_4
play the rosbag
"""


import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA

rospy.init_node("marker_visualization")
landmarks = rospy.get_param("~landmarks")
# landmarks = {"pole1": [0, 2.44, -5]}

pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

seq = 0
for i in landmarks.keys():
    m = Marker()
    m.header.frame_id = "map"
    m.id = seq
    m.type = m.CYLINDER
    m.ns = ""
    m.pose.position = Point(landmarks[i][0], landmarks[i][1], 0)
    m.pose.orientation.w = 1
    m.scale = Vector3(0.5,0.5,1)
    m.color = ColorRGBA(1,0,0,1)

    rospy.sleep(0.5)
    pub.publish(m)

    seq += 1

rospy.sleep(0.5)
m = Marker()
m.header.frame_id = "map"
m.id = seq
m.type = m.CUBE
m.ns = ""
m.pose.position = Point(-1,0,0)
m.pose.orientation.w = 1
m.scale = Vector3(2,10,1)
m.color = ColorRGBA(0.2,0,0.1,1)
pub.publish(m)