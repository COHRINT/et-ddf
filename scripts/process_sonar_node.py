#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from scipy import stats
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from etddf.srv import SetSonarSettings
from minau.msg import SonarTargetList, SonarTarget
from etddf.msg import NetworkEstimate, AssetEstimate
import os
import tf
import math


rospy.init_node("process_sonar_node")

class ProcessSonar:
    def __init__(self):
        self.own_yaw = None
        ownship = "etddf/estimate" + rospy.get_namespace()[:-1]
        otherAsset = "etddf/estimate/bluerov2_4"
        network = "etddf/estimate/network"

        #subscribes to the estimates and the sonar data
        rospy.Subscriber(ownship,Odometry,self.own_pose_callback)
        rospy.Subscriber(network,NetworkEstimate,self.network_callback)
        rospy.Subscriber("sonar_filtered",LaserScan,self.sonar_callback)

        self._sonar_targets_publisher = rospy.Publisher(
                'ping_360_target',
                SonarTargetList, queue_size=10)
        
        

        pole_x = rospy.get_param("~pole_x",10)
        pole_y = rospy.get_param("~pole_y",0)
        # print(pole_x)
        # structure of this will be : [[[x1,y1],[x2,y2]...]]
        # each element represents a group that is one detection
        self.pole_location = np.array([pole_x,pole_y])         # the z is just a place holder because we don't care about that location
        self.detections = []
        self._sequential_observation_id = 1
        self.num_wo_hits = []
    def own_pose_callback(self,msg):
        """
        Takes from etddf estimate the ownship pose estimate

        Args:
            msg (Odometry): etddf ownship estimate
        """
        self.own_pose = msg
        self.own_position = msg.pose.pose.position
        (r, p, self.own_yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # print(self.own_yaw)
    def network_callback(self,msg):
        """Takes etddf estimate of the other asset

        Args:
            msg (Odometry): etddf estimate of other asset
        """
        self.network = msg
    # use: multivariate_normal.cdf to find probablity of the rov to be there(will give the probablity that the rov is there or less)
    def calc_point(self,angle,range):
        """ Based the the assets ownship position and the angle and range it detects something
        it returns the x,y of where the detection was.

        Args:
            angle (float): angle the rov sees the detection 
            range (float): distance to detection

        Returns:
            [float,float]: the x,y of where the detection is
        """
        x = range*np.cos(angle+self.own_yaw) + self.own_position.x
        y = range*np.sin(angle+self.own_yaw) + self.own_position.y
        return [x,y]
    
    def is_matched(self,det, group):
        """Takes in a detection(x,y) and returns if it is close enough
        to another detection in a group of detections to be in that group too.

        Args:
            det ([float, float]): x,y of detection
            group ([[float,float],[float,float]]): list of x,yf. corrdinates that belong to the same detection

        Returns:
            [boolean]: if the detection belongs to that group or not
        """
        for i in range(len(group)):
            if np.linalg.norm([det[0]-group[i][0],det[1]-group[i][1]]) < .5:
                return True
        return False
    
    def avg(self, detect):
        """Takes a list of x,y corrdinates and averages them

        Args:
            detect (list of [float,float]): list of x,y that belong to a detection

        Returns:
            [float,float]: the average x,y of the detections
        """
        count = 0
        x = 0
        y = 0
        for i in range(len(detect)):
            x+= detect[i][0]
            y+= detect[i][1]
            count+= 1
        count = float(count)
        avg_x = x/count
        avg_y = y/count
        return [avg_x,avg_y]

    def find_own_location(self,detection):
        """Assumes detection is the pole and determines where the rovs ownship position would be if that were the pole

        Args:
            detection ([float,float,float]): x,y,z or detection based on ownship own estimate at the time

        Returns:
            [np.array(float,float,float)]: Ownship location if that is indeed the pole
        """
        own_pose = np.array([self.own_position.x,self.own_position.y])
        diff = self.pole_location - detection
        return own_pose+diff

    def prob(self,odom,detection):
        """Determines probablity that a distribution could have a point with the malanobis distance or farther away as the detection

        Args:
            odom (Odometry): Odometry of the distribution we are checking the detection against
            detection ([float,float,float]): Location of detection we are checking

        Returns:
            [float]: the probablity that the distribution could have a point beloning to it that has a
            malonobis distance equal to or farther away as the detection
        """
        nav_cov = np.array(odom.pose.covariance).reshape(6,6)
        cov = np.zeros((2,2))
        cov[:2,:2] = nav_cov[:2,:2]
        estimate = np.array([odom.pose.pose.position.x,odom.pose.pose.position.y])
        m_dist_x = np.dot((detection-estimate).transpose(),np.linalg.inv(cov))
        m_dist_x = np.dot(m_dist_x, (detection-estimate))
        return (1-stats.chi2.cdf(m_dist_x, 2))

    def calc_yaw_and_range(self,detection):
        """Calculate angle and range of the detection

        Args:
            detection ([float,float,float]): Location of detection

        Returns:
            float,float: angle, and range of detection reletive to the rovs own position
        """
        detection = np.array(detection)
        own_pose = np.array([self.own_position.x,self.own_position.y])
        range_to = np.linalg.norm(detection-own_pose)
        angle = np.arctan2(detection[1]-own_pose[1],detection[0]-own_pose[0])
        return angle,range_to




    def publish_detection(self,id, detection,uuv_class):
        """Publishes the detection with OL message type

        Args:
            id (string): asset id
            detection ([float,float,float]): where the detection was
        """
        target = SonarTarget()
        target.id = id
        target.elevation_rad = 0.0
        target.elevation_variance = 0.0
        target_yaw_inertial,range_to_target = self.calc_yaw_and_range(detection)
        target.bearing_rad = target_yaw_inertial
        target.bearing_variance = 0.0
        target.range_m = range_to_target
        target.range_variance
        target.associated = True
        target.type = None
        if id == "pole":
            target.type = SonarTarget.TARGET_TYPE_OBJECT
        else:
            target.type = SonarTarget.TARGET_TYPE_UUV
        target.uuv_classification = uuv_class
        target_list = SonarTargetList()
        target_list.header.stamp = rospy.Time.now()
        target_list.header.frame_id = os.path.join(rospy.get_namespace(),"baselink")
        target_list.header.seq = self._sequential_observation_id
        self._sequential_observation_id += 1
        target_list.targets.append(target)
        self._sonar_targets_publisher.publish(target_list)



    def determine_detection(self):
        """
        Looks through the current detections and sees if any of them have not been seen in a while,
        if so it remove them from the detection list, averages the detection and sees the probablity it is
        the other blue asset
        """
        detection = None
        # if I haven't seen the object in a bit, I am going to publish its position and pop 
        for j in range(len(self.num_wo_hits)):
            if self.num_wo_hits[j] > 3 or len(self.detections[j])>30:
                self.num_wo_hits.pop(j)
                detection = self.detections.pop(j)
                break
                
        if detection != None:
            # print(detection)
            avg_detection = self.avg(detection)
            
            #finds mihalomis distance and then
            #using chi squared test to see how proble it is that the other blue asset is there.
            p_network = []
            for i in range(len(self.network.assets)):
                if self.network.assets[i].name != rospy.get_namespace()[1:-1]:
                    p = self.prob(self.network.assets[i].odom,avg_detection)
                    p_network.append([self.network.assets[i].name,p])

            p_asset = p_network[0]
            for i in range(len(p_network)):
                if p_network[i][1] > p_asset[1]:
                    p_asset = p_network[i]

           
            #assumes the detection is the pole then based on that detection, calculates where the ownship position would be
            #then it finds the mahlomis distance based on the ownship mean and covariance
            own_pole_location = self.find_own_location(avg_detection)
            p_pole = self.prob(self.own_pose,own_pole_location)
          
            location = " at " + str(avg_detection[0]) + ', ' + str(avg_detection[1])
            # print(p_pole)
            # print(p_asset)
            asset_id = None
            uuv_class = None
            if p_pole < .05 and p_asset[1] < .05:
                # print('I think I just saw the red asset'+location)
                asset_id = 'red_asset'
                uuv_class = SonarTarget.UUV_CLASS_RED
            else:
                if p_pole > p_asset[1]:
                    # print('I think I just saw the pole'+location)
                    asset_id = 'pole'
                    uuv_class = SonarTarget.UUV_CLASS_UNKNOWN
                else:
                    # print('I think I just saw the other blue asset'+location)
                    asset_id = p_asset[0]
                    uuv_class = SonarTarget.UUV_CLASS_BLUE
            
            self.publish_detection(asset_id,avg_detection,uuv_class)



    def sonar_callback(self,msg):
        """ Takes in the filtered sonar data and records it if there is a detection.
        Groups detections together by how close they are to each other.

        Args:
            msg (LaserScan): The filtered sonar data
        """
        #makes sure we have ownship
        if self.own_yaw == None:
            return
        angle_diff = 2*np.pi / len(msg.ranges)
        angles = [-np.pi + angle_diff*i for i in range(len(msg.ranges))]
        for i in range(len(self.num_wo_hits)):
            self.num_wo_hits[i]+=1
        #loops through data and determines if there was a detection and where is should group that detection
        for i in range(len(msg.ranges)):
            if msg.ranges[i] != -1:
                detected_point = self.calc_point(angles[i],msg.ranges[i])
                matched = False
                for j in range(len(self.detections)):
                    if self.is_matched(detected_point,self.detections[j]):
                        matched = True
                        self.detections[j].append(detected_point)
                        self.num_wo_hits[j] = 0
                if not matched:
                    self.detections.append([detected_point])
                    self.num_wo_hits.append(0)
        #calls determine_detection to see if detection is done
        self.determine_detection()



if __name__ == "__main__":
    sp = ProcessSonar()
    while not rospy.is_shutdown():
        rospy.spin()


