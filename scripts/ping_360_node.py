#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from scipy.stats import multivariate_normal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from etddf.srv import SetSonarSettings, GetSonarSettings
from etddf.msg import SonarSettings
import tf
import copy
 
rospy.init_node("ping_360_sonar")
 
 
# use: multivariate_normal.cdf to find probablity of the rov to be there(will give the probablity that the rov is there or less)
def normalize_ang(ang):
   while ang < 0:
       ang+=2*np.pi
   while ang > 2*np.pi:
       ang-=2*np.pi
   return ang
 
 
class SonarPing:
   """
   Takes the raw sonar data that shows all 360 of pings every update and makes it so the sonar only sees
   whats in the sonars range of view. Also makes a service that can change the settings of the sonar
   """
   def __init__(self):
       """
       Subscribes to the etddf estimates and the raw sonar data
       """
       self.own_yaw = None
       ownship = "etddf/estimate" + rospy.get_namespace()[:-1]
 
       #subscribes to the estimates and the sonar data
       rospy.Subscriber(ownship,Odometry,self.own_pose_callback)
       rospy.Subscriber("ping360raw",LaserScan,self.sonar_callback)
 
       #Initiates the publishers
       self.filtered_sonarpub = rospy.Publisher("sonar_filtered",LaserScan,queue_size=10)
       self.estimatepub = rospy.Publisher("sonar_pose",Odometry,queue_size=10)
       self.visualpub = rospy.Publisher("visual_pub",LaserScan,queue_size=10)
 
       #Initiate the service to change the settings of the sonar
       rospy.Service("set_sonar_settings",SetSonarSettings,self.set_settings)
       rospy.Service("get_sonar_settings",GetSonarSettings,self.get_settings)
 
       #Give the settings for when the sonar is at max range and set to 360
       self.sonar = None
       self.start_time = rospy.get_rostime().secs * 10**9 + rospy.get_rostime().nsecs
       self.range = rospy.get_param('~range',default=10.0)
       self.min_angle = -np.pi
       self.max_angle = np.pi
       self.time = 13.*self.range/24+95./12
       self.two_degrees = np.pi/90
       self.angle_range = np.array([-np.pi,-np.pi+self.two_degrees])               #Can see 1 degree increments
       self.initial_range = np.array([-np.pi,-np.pi+self.two_degrees])
       self.fraction_angles = ((self.max_angle-self.min_angle)/(2*np.pi))
       self.full_scan = True
       self.time_per_scan = int(((self.time/360)/self.fraction_angles) * 10**9)
       # print(self.time_per_scan)
       self.scan_positive = True
       self.count = 0
   def own_pose_callback(self,msg):
       """
       Takes from etddf estimate the ownship pose estimate
 
       Args:
           msg (Odometry): etddf ownship estimate
       """
       self.own_pose = msg
       (r, p, self.own_yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
   def determine_angle_range(self,time):
       """
       Determine the angle range the sonar can see at a current time
 
       Args:
           time (stamp): time the sonar measurment was stamped with
 
       Returns:
           Boolean : True if the range didn't change, False if it did
       """
       converted_time = time.secs*10**9 + time.nsecs
       # print(converted_time - self.start_time)
       # print(self.time_per_scan)
       # print('')
       # print(rospy.get_rostime())
       # print((converted_time%(self.time*10**9))/self.time_per_scan)
       # print('')
       if converted_time - self.start_time  >= (self.time_per_scan*2):
           # print(self.count)
           self.count = 0
           # print(self.time_per_scan)
           # print(self.angle_range[0])
           # if self.angle_range[0] == self.initial_range[0]:
           #     print(rospy.get_rostime())
           while self.start_time < converted_time:
               self.start_time += self.time_per_scan*2
               self.count +=1
           if self.count > 30:
               return
           if self.scan_positive:
               self.angle_range[0] = self.angle_range[1]
               self.angle_range[1] = self.angle_range[0] + self.count*self.two_degrees
           else:
               self.angle_range[1] = self.angle_range[0]
               self.angle_range[0] = self.angle_range[1] - self.count*self.two_degrees
          
           if self.angle_range[1] > self.max_angle and self.angle_range[0] < self.max_angle:
               if self.full_scan:
                   self.angle_range = np.array([-np.pi,-np.pi+self.two_degrees*self.count])
               elif self.scan_positive:
                   self.scan_positive = False
          
           if self.angle_range[0] < self.min_angle and self.angle_range[1]>self.min_angle:
               if self.full_scan:
                   self.angle_range = np.array([np.pi-self.two_degrees,np.pi*self.count])
               elif not self.scan_positive:
                   self.scan_positive = True
          
           if self.angle_range[0] <= -np.pi and self.angle_range[1]<=-np.pi:
               self.angle_range+=2*np.pi
           if self.angle_range[0]>= np.pi and self.angle_range[1]>=np.pi:
               self.angle_range-=2*np.pi
           return False
       else:
           self.count+=1
           return True
          
   def sonar_callback(self,msg):
       """
       Takes in the raw sonar measurment and publishes what the sonar can actually see given during a given time
 
       Args:
           msg (LaserScan): [Raw sonar data]
       """
       if self.own_yaw == None:
           return
 
       self.sonar = msg
       #this pulls from the message the horizontal samples and vertical samples
       #assumes the sonar is 360
       self.horz_count = int(round(np.pi*2 / msg.angle_increment))
       self.vert_count = int(len(msg.ranges)/self.horz_count)
       #this makes an array, one spot for every horizontal angle, and puts the shortest range in that value
       self.detect = [-1 for i in range(self.horz_count)]
       self.visual = [10 for i in range(self.horz_count)]
       self.angles = [-np.pi + i*msg.angle_increment for i in range(self.horz_count)]
       for i in range(len(msg.ranges)):
           if msg.ranges[i]<=msg.range_max:
               if self.detect[i%self.horz_count] == -1 or self.detect[i%self.horz_count]>msg.ranges[i]:
                   self.detect[i%self.horz_count] = msg.ranges[i]
 
   def set_settings(self,req):
       """
       Service that changes the settings of the sonar
 
       Args:
           req (SonarSettings): the settings given by the service call
 
       Returns:
           [Boolean]: [Determines whether the call was successful or not]
       """
       # print('hello')
       if req.settings.range >= 50:
           self.range = 50
       elif req.settings.range <= 2:
           self.range = 2
       else:
           self.range = req.settings.range
       if req.settings.min_angle < -np.pi:
           self.min_angle = -np.pi
       else:
           self.min_angle = req.settings.min_angle
       if req.settings.max_angle > np.pi:
           self.max_angle = np.pi
       else:
           self.max_angle = req.settings.max_angle
       time = 13.*self.range/24+95./12
       self.time = time
       angleRatio = 2*np.pi/(self.max_angle-self.min_angle)
 
 
       #for situation when looking for pole to know when to look at a larger angle range
       max_diff = normalize_ang(self.max_angle - self.angle_range[0])
       min_diff = normalize_ang(self.angle_range[0] - self.min_angle)
       time_to_scan_all = None
       if max_diff < min_diff:
           time_to_scan_all = (time * max_diff)/(2*np.pi)
       else:
           time_to_scan_all = (time * min_diff)/(2*np.pi)
 
       self.time = self.time / angleRatio
       if self.time > time_to_scan_all:
           time_to_scan_all += self.time
       self.fraction_angles = ((self.max_angle-self.min_angle)/(2*np.pi))
       self.time_per_scan = int(((self.time/360)/self.fraction_angles) * 10**9)
       # print(self.time_per_scan)
       if(self.min_angle!= -np.pi or self.max_angle!=np.pi):
           self.full_scan = False
           ang_avg = (self.min_angle+self.max_angle)/2
           # print(self.angle_range[0]-ang_avg)
           # print('')
           # print('')
           if self.angle_range[0] > self.max_angle or self.angle_range[0] < self.min_angle:      #if current angle is outside the range, check which way to turn
               if self.angle_range[0]-ang_avg < 0:
                   self.scan_positive = True
               else:
                   self.scan_positive = False
       else:
           self.full_scan = True
      
       if self.min_angle > self.max_angle:
           if self.scan_positive:
               self.scan_positive = False
           else:
               self.scan_positive = True
       return time_to_scan_all
   def get_settings(self,req):
       settings = SonarSettings()
       settings.max_angle = self.max_angle
       settings.min_angle = self.min_angle
       settings.range = self.range
       return settings
   def run(self):
       rate = rospy.Rate(10)
       while not rospy.is_shutdown():
           rate.sleep()
           if self.determine_angle_range(rospy.get_rostime()) or self.sonar == None:
               continue
           # print(self.angle_range)
           ranges = [-1 for i in range(self.horz_count)]
           visual_ranges = [-1 for i in range(self.horz_count)]
           intensities = [0 for i in range(self.horz_count)]
           for i in range(len(self.detect)):
               if self.angles[i] >= self.angle_range[0] and self.angles[i] <= self.angle_range[1]:
                   ranges[i] = self.detect[i]
                   visual_ranges[i] = self.visual[i]
          
           cropped_view = copy.deepcopy(self.sonar)
           cropped_view.ranges = ranges
           cropped_view.intensities = intensities
           cropped_view.range_max = self.range
 
           self.filtered_sonarpub.publish(cropped_view)
 
           visual_view = copy.deepcopy(self.sonar)
           visual_view.ranges = visual_ranges
           visual_view.intensities = intensities
           visual_view.range_max = 15
 
           self.visualpub.publish(visual_view)
 
 
 
 
 
 
if __name__ == "__main__":
   sp = SonarPing()
   sp.run()
   rate = rospy.Rate(1)
   while not rospy.is_shutdown():
       rate.sleep()