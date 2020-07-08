#!/usr/bin/env python
from __future__ import division
import rospy
from nav_msgs.msg import Odometry
import tf
from etddf.srv import SetSonarSettings, GetSonarSettings
from etddf.msg import SonarSettings
from minau.msg import SonarTargetList, SonarTarget
import numpy as np

def convert_ros_time(r):
    return r.secs + r.nsecs*10**(-9)

class ScannerControl:
    def __init__(self):
        self.own_yaw = None
        ownship = "etddf/estimate" + rospy.get_namespace()[:-1]
        rospy.wait_for_service("set_sonar_settings")
        self.set_sonar = rospy.ServiceProxy("set_sonar_settings",SetSonarSettings)
        self.get_sonar = rospy.ServiceProxy("get_sonar_settings",GetSonarSettings)
        rospy.Subscriber(ownship,Odometry,self.pose_callback)
        rospy.Subscriber("ping_360_target",SonarTargetList,self.check_for_pole)
        self.threshold = rospy.get_param("threshold",2)
        pole_x = rospy.get_param("~pole_x",10)
        pole_y = rospy.get_param("~pole_y",0)
        # print(pole_x)
        self.seen_pole = False
        self.previous = None
        self.pole_loc = [pole_x,pole_y]
        self.thirty_degrees = np.pi/6.
    def get_dist(self,loc):
        """Gets the distance between the rovs ownship position and the pole's location

        Args:
            loc ([float,float]): Rovs ownship position estimate

        Returns:
            [float]: Distance to pole
        """
        diffx = loc[0]-self.pole_loc[0]
        diffy = loc[1]-self.pole_loc[1]
        return np.linalg.norm([diffx,diffy])

    def pose_callback(self,msg):
        """
        Takes from etddf estimate the ownship pose estimate

        Args:
            msg (Odometry): etddf ownship estimate
        """
        self.own_pose = msg
        (r, p, self.own_yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    def check_for_pole(self,msg):
        for i in range(len(msg.targets)):
            print("I saw a " + msg.targets[i].id)
            if msg.targets[i].id == "pole":
                self.seen_pole = True

    def run(self):
        """
        Checks if the covariance is too big and if so points the sonar toward the pole
        """
        rate = rospy.Rate(5)
        while True:
            if self.own_yaw == None:
                rate.sleep()
                continue
            if self.own_pose.pose.covariance[0] + self.own_pose.pose.covariance[7] > self.threshold:
                print("Passed threshold")
                self.previous = self.get_sonar().settings
                distance = self.get_dist([self.own_pose.pose.pose.position.x,self.own_pose.pose.pose.position.y])
                relative_diff = [self.pole_loc[0]-self.own_pose.pose.pose.position.x,self.pole_loc[1]-self.own_pose.pose.pose.position.y]
                angle = np.arctan2(relative_diff[1],relative_diff[0])
                # print(relative_diff)
                # print(angle)
                angle = angle - self.own_yaw
                # print(angle)
                self.seen_pole = False
                num = 1
                while not self.seen_pole:
                    settings = SonarSettings()
                    settings.range = distance+self.own_pose.pose.covariance[0]
                    settings.min_angle = angle-(self.thirty_degrees*num)
                    settings.max_angle = angle+(self.thirty_degrees*num)
                    wait_time = self.set_sonar(settings).time-.5
                    start_time = convert_ros_time(rospy.get_rostime())
                    rate2 = rospy.Rate(10)
                    while (convert_ros_time(rospy.get_rostime()) - start_time < wait_time) and not self.seen_pole:
                        rate2.sleep()
                    #if the pole hasn't been seen, check a wider area
                    if not self.seen_pole:
                        print('Expanding Range')
                        num+=1
                    else:
                        print('I saw the pole!')
                        self.set_sonar(self.previous)




rospy.init_node("scanner_control")


if __name__ == "__main__":
    rate = rospy.Rate(.2)
    sc = ScannerControl()
    sc.run()
    while not rospy.is_shutdown():
        rate.sleep()

