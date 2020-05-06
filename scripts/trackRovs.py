#!/usr/bin/env python
from __future__ import division
import rospy
from minau.srv import ArmControl, SetHeadingVelocity
from minau.msg import ControlStatus
from std_msgs.msg import Header
from etddf.msg import Sonar
from nav_msgs.msg import Odometry
import numpy as np
import math
import tf
from geometry_msgs.msg import Vector3


#publish to message type you created, do more testing
class TrackingRov:
    def __init__(self):
        self.pose = None
        self.yaw = None
        rospy.Subscriber("pose_gt", Odometry, self.pose_callback)
    def pose_callback(self, msg):
        self.pose = msg
        (r, p, self.yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

class EnemyRov:
    def __init__(self):
        self.pose = None
        rospy.Subscriber("/rexrov2/pose_gt",Odometry,self.pose_callback)
    def pose_callback(self,msg):
        self.pose = msg

def distance(A,B):
    return math.sqrt((A.pose.pose.pose.position.x-B.pose.pose.pose.position.x)**2+(A.pose.pose.pose.position.y-B.pose.pose.pose.position.y)**2+(A.pose.pose.pose.position.z-B.pose.pose.pose.position.z)**2)


def checkZone(blue,red):
    #normalize coordinates so blue rov is at origin
    normX = red.pose.pose.pose.position.x-blue.pose.pose.pose.position.x
    normY = red.pose.pose.pose.position.y-blue.pose.pose.pose.position.y
    normZ = red.pose.pose.pose.position.z-blue.pose.pose.pose.position.z
    #calculate distance in 3d and just in xy
    dist = np.linalg.norm([normX,normY,normZ])
    distXY = np.linalg.norm([normX,normY])
    #in this new coordinate system find 'yaw' so we can compare it to yaw of blue rov
    estYaw =  np.arctan2(normY,normX)
    normYaw = blue.yaw - estYaw
    #want to make sure angle is between -pi and pi
    if normYaw > np.pi:
        normYaw-=2*np.pi
    elif normYaw < -np.pi:
        normYaw+=2*np.pi
    normElev = np.arctan2(normZ,distXY)
    # the sensor can see 70 degrees horizontally(35 each way) and 12 vertically(6 each way)
    horzAng = 35 * np.pi /180
    vertAng = 6 * np.pi /180
    #Check if in range, in horizontal range, and in vertical range
    if dist <= 40 and np.abs(normYaw) <= horzAng and np.abs(normElev) <= vertAng:
        pub = rospy.Publisher('sonarData',Sonar,queue_size=10)
        s = Sonar()
        s.azimuth = normYaw
        s.elevation = normElev
        s.range = dist
        s.asset_name = 'rexrov2'
        rospy.loginfo(s)
        pub.publish(s)
    


rospy.init_node("track_rov")
tr = TrackingRov()
er = EnemyRov()
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    if (tr.pose != None) and (er.pose != None):
        checkZone(tr,er)
        #print(tr.yaw)
    rate.sleep()



