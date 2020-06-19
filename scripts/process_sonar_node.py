#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf

rospy.init_node("process_sonar")



class SonarProcess:
    def __init__(self):
        self.yaw = None
        rospy.Subscriber("/bluerov2_3/etddf/estimate/bluerov2_3",Odometry,self.pose_callback)
        rospy.Subscriber("/bluerov2_3/sensor/sonar",LaserScan,self.sonar_callback)
        self.estimatepub = rospy.Publisher("/bluerov2_3/sonar_pose",Odometry,queue_size=10)
    def pose_callback(self,msg):
        self.pose = msg
        (r, p, self.yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    def sonar_callback(self,msg):
        if self.yaw == None:
            return
        #this pulls from the message the horizontal samples and vertical samples
        #assums the sonar is 360
        self.horz_count = int(round(np.pi*2 / msg.angle_increment)+1)
        self.vert_count = int(len(msg.ranges)/self.horz_count)
        #this makes an array, one spot for every horizontal angle, and puts the shortest range in that value
        self.detect = [-1 for i in range(self.horz_count)]
        self.angles = [-np.pi + i*msg.angle_increment for i in range(self.horz_count)]
        for i in range(len(msg.ranges)):
            if msg.ranges[i]<=msg.range_max:
                if self.detect[i%self.horz_count] == -1 or self.detect[i%self.horz_count]>msg.ranges[i]:
                    self.detect[i%self.horz_count] = msg.ranges[i]
        print(self.detect)
        sonar_est = self.est_pose()
        print(sonar_est)
        if sonar_est != None:
            pose = self.pose
            pose.pose.pose.position.x = sonar_est[0]
            pose.pose.pose.position.y = sonar_est[1]
            print("About to publish")
            self.estimatepub.publish(pose)
    def est_pose(self):
        estimated_x = []
        estimated_y = []
        for i in range(self.horz_count):
            if self.detect[i] != -1:
                estimated_pose = self.calc_point(self.angles[i],self.detect[i])
                estimated_x.append(estimated_pose[0])
                estimated_y.append(estimated_pose[1])
        if estimated_x == []:
            return None
        combined_x = np.average(estimated_x)
        combined_y = np.average(estimated_y)
        print([combined_x,combined_y])
        return [combined_x,combined_y]

    def calc_point(self,ang,dist):
        #positive angles counterclockwise as viewed from the top
        pos = [-dist*np.cos(ang+self.yaw),-dist*np.sin(ang+self.yaw)]
        return pos



sp = SonarProcess()
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    rate.sleep()