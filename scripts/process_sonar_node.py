#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from scipy.stats import multivariate_normal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from etddf.srv import SetSonarSettings
import tf

rospy.init_node("process_sonar")


# use: multivariate_normal.cdf to find probablity of the rov to be there(will give the probablity that the rov is there or less)


class SonarProcess:
    """
    Takes the raw sonar data that shows all 360 of pings every update and makes it so the sonar only sees 
    whats in the sonars range of view. Also makes a service that can change the settings of the sonar
    """
    def __init__(self):
        """
        Subscribes to the etddf estimates and the raw sonar data
        """
        self.own_yaw = None
        ownship = "etddf/estimate/bluerov2_3"
        otherAsset = "etddf/estimate/bluerov2_4"
        #determines which asset this is
        if rospy.get_namespace() == '/bluerov2_4/':
            ownship,otherAsset = otherAsset,ownship

        #subscribes to the estimates and the sonar data
        rospy.Subscriber(ownship,Odometry,self.own_pose_callback)
        rospy.Subscriber(otherAsset,Odometry,self.other_pose_callback)
        rospy.Subscriber("ping360raw",LaserScan,self.sonar_callback)

        #Initiates the publishers
        self.filtered_sonarpub = rospy.Publisher("sonar_filtered",LaserScan,queue_size=10)
        self.estimatepub = rospy.Publisher("sonar_pose",Odometry,queue_size=10)

        #Initiate the service to change the settings of the sonar
        rospy.Service("set_sonar_settings",SetSonarSettings,self.set_settings)


        #Give the settings for when the sonar is at max range and set to 360
        self.start_time = rospy.get_rostime().secs * 10**9 + rospy.get_rostime().nsecs
        self.range = 50
        self.min_angle = -np.pi
        self.max_angle = np.pi
        self.fixed = False
        self.time = 35.
        self.angle_range = np.array([-np.pi,-np.pi+(np.pi/360)])               #Can see 1 degree increments
        self.fraction_angles = ((self.max_angle-self.min_angle)/(2*np.pi))
        self.full_scan = True
        self.time_per_scan = ((self.time/360)/self.fraction_angles) * 10**9
        self.scan_positive = 1
    def own_pose_callback(self,msg):
        """
        Takes from etddf estimate the ownship pose estimate

        Args:
            msg (Odometry): etddf ownship estimate
        """
        self.own_pose = msg
        (r, p, self.own_yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    def other_pose_callback(self,msg):
        """Takes etddf estimate of the other asset

        Args:
            msg (Odometry): etddf estimate of other asset
        """
        self.other_pose = msg
    def determine_angle_range(self,time):
        """
        Determine the angle range the sonar can see at a current time

        Args:
            time (stamp): time the sonar measurment was stamped with

        Returns:
            Boolean : True if the range didn't change, False if it did
        """
        converted_time = time.secs*10**9 + time.nsecs
        if converted_time - self.start_time  >= self.time_per_scan:
            self.start_time = converted_time
            if self.scan_positive % 2 == 1:
                self.angle_range += np.pi/360. 
            else:
                self.angle_range -= np.pi/360.
            
            if self.angle_range[1] > self.max_angle:
                if self.full_scan:
                    self.angle_range = np.array([-np.pi,-np.pi+(np.pi/360)])
                else:
                    self.scan_positive+=1
            
            if self.angle_range[0] < self.min_angle:
                if self.full_scan:
                    self.angle_range = np.array([np.pi-(np.pi/360),np.pi])
                else:
                    self.scan_positive+=1
            return False
        else:
            return True
            
    def sonar_callback(self,msg):
        """
        Takes in the raw sonar measurment and publishes what the sonar can actually see given during a given time

        Args:
            msg (LaserScan): [Raw sonar data]
        """
        if self.own_yaw == None:
            return
    
        if self.determine_angle_range(msg.header.stamp):
            return
        #this pulls from the message the horizontal samples and vertical samples
        #assumes the sonar is 360
        self.horz_count = int(round(np.pi*2 / msg.angle_increment)+1)
        self.vert_count = int(len(msg.ranges)/self.horz_count)
        #this makes an array, one spot for every horizontal angle, and puts the shortest range in that value
        self.detect = [-1 for i in range(self.horz_count)]
        ranges = [-1 for i in range(self.horz_count)]
        intensities = [0 for i in range(self.horz_count)]
        self.angles = [-np.pi + i*msg.angle_increment for i in range(self.horz_count)]
        for i in range(len(msg.ranges)):
            if msg.ranges[i]<=msg.range_max:
                if self.detect[i%self.horz_count] == -1 or self.detect[i%self.horz_count]>msg.ranges[i]:
                    self.detect[i%self.horz_count] = msg.ranges[i]
        
        for i in range(len(self.detect)):
            if self.angles[i] >= self.angle_range[0] and self.angles[i] <= self.angle_range[1]:
                ranges[i] = self.detect[i]
        
        cropped_view = msg
        cropped_view.ranges = ranges
        cropped_view.intensities = intensities
        cropped_view.range_max = self.range

        self.filtered_sonarpub.publish(cropped_view)



        # sonar_est = self.est_pose()
        # if sonar_est != None:
        #     pose = self.own_pose
        #     pose.pose.pose.position.x = sonar_est[0]
        #     pose.pose.pose.position.y = sonar_est[1]
        #     self.estimatepub.publish(pose)
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
        return [combined_x,combined_y]

    def calc_point(self,ang,dist):
        #positive angles counterclockwise as viewed from the top
        pos = [-dist*np.cos(ang+self.own_yaw),-dist*np.sin(ang+self.own_yaw)]
        return pos
    def set_settings(self,req):
        """
        Service that changes the settings of the sonar

        Args:
            req (SetSonarSettings.srv): the settings given by the service call

        Returns:
            [Boolean]: [Determines whether the call was successful or not]
        """
        if req.range >= 50:
            self.range = 50
        elif req.range <= 2:
            self.range = 2
        else:
            self.range = req.range
        if req.min_angle < -np.pi:
            self.min_angle = -np.pi
        else:
            self.min_angle = req.min_angle
        if req.max_angle > np.pi:
            self.max_angle = np.pi
        else:
            self.max_angle = req.max_angle
        self.fixed = req.fixed_angle
        time = 13.*self.range/24+95./12
        self.time = time
        angleRatio = 2*np.pi/(self.max_angle-self.min_angle)
        self.time = self.time / angleRatio
        self.fraction_angles = ((self.max_angle-self.min_angle)/(2*np.pi))
        self.time_per_scan = ((self.time/360)/self.fraction_angles) * 10**9
        if(self.min_angle!= -np.pi or self.max_angle!=np.pi):
            self.full_scan = False
        else:
            self.full_scan = True
        return True



if __name__ == "__main__":
    sp = SonarProcess()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()