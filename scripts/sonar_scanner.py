#!/usr/bin/env python


"""

    Emulates ping 360 scanne, rotates the scan angle and if the red asset is spotted points the sonar toward it
    """
from __future__ import division
import rospy
import numpy as np
from etddf.srv import Detection
from ping360_sonar.srv import SetSonarSettings
from ping360_sonar.msg import SonarSettings
from std_msgs.msg import Int16


class SonarScanner():
    def __init__(self):
        self.sonar_range = rospy.get_param("~sonar_range",default=10)
        self.angles_per_scan = 18

        timing_eq = [0.54166667,7.91666667]      #equation for how long it takes to scan full 360([m,b])

        self.scan_time = timing_eq[0]*self.sonar_range + timing_eq[1]
        self.scan_time = self.scan_time * (self.angles_per_scan/360.0)
        self.set_settings = rospy.ServiceProxy("ping360_node/sonar/set_sonar_settings",SetSonarSettings)

        self.num_scans = int(360/self.angles_per_scan)
        self.angle = 0
        self.full_angle_range = range(20)
        self.angle_range = self.full_angle_range
        self.use_360 = True


        rospy.Service("red_asset_detect_angle", Detection, self.detection_angle)
        self.pub_angle = rospy.Publisher("sonar_angle",Int16,queue_size=10)
    
    def detection_angle(self,req):
        ang = req.angle
        #when the rov spots the asset it turns toward it so it will be centered at 0
        self.detection_time = rospy.get_rostime().secs
        self.angle = 0
        self.angle_range = [18,19,0,1]
        self.dir = True   #true for positive scans
        self.use_360 = False
        return True
    
    def normalize_ang_degree(self,ang):
        if ang > 180:
            return ang-360
        return ang
    
    def run(self):
        settings = SonarSettings()
        settings.range = 10
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            settings.min_angle_deg = self.normalize_ang_degree(self.angle * self.angles_per_scan)
            self.pub_angle.publish(Int16(self.angle * self.angles_per_scan))
            settings.max_angle_deg = self.normalize_ang_degree(((self.angle+1)%self.num_scans) * self.angles_per_scan)

            self.set_settings(settings)
            if self.use_360:
                self.angle = (self.angle+1)%self.num_scans
            else:
                if self.dir:
                    if self.angle == self.angle_range[-1]:
                        self.dir = False
                    else:
                        self.angle = (self.angle+1)%(self.num_scans)
                if not self.dir:
                    if self.angle == self.angle_range[0]:
                        self.dir = True
                        self.angle = (self.angle+1)%(self.num_scans)
                    else:
                        self.angle = (self.angle-1)%(self.num_scans)
                if rospy.get_rostime().secs-self.detection_time > 5:
                    self.angle_range = self.full_angle_range
                    self.use_360 = True



            rospy.sleep(self.scan_time)


rospy.init_node("sonar_scanner")

if __name__ == "__main__":
    sc = SonarScanner()
    sc.run()





