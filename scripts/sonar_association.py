#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from minau.msg import SonarTargetList, SonarTarget
import tf
import numpy as np
from cuprint.cuprint import CUPrint

class SonarAssociator:

    def __init__(self):
        
        self.cuprint = CUPrint("SonarAssociator")
        self.association_threshold_distance = 1

        self.landmark_dict = rospy.get_param("~landmarks")

        # pose_topic = "etddf/estimate" + rospy.get_namespace()[:-1]
        pose_topic = rospy.get_namespace()[:-1] + "/pose_gt"
        rospy.Subscriber(pose_topic, Odometry, self.pose_callback)
        rospy.wait_for_message(pose_topic, Odometry)

        self.pub = rospy.Publisher("sonar_processing/target_list/associated", SonarTargetList, queue_size=10)

        sonar_topic = "sonar_processing/target_list"
        rospy.Subscriber(sonar_topic, SonarTargetList, self.sonar_callback)

        self.cuprint("Loaded")

    def pose_callback(self, msg):
        self.pose = msg

    def sonar_callback(self, msg):
        for i in range(len(msg.targets)):

            ori = self.pose.pose.pose.orientation
            _, _, current_yaw = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            current_x = self.pose.pose.pose.position.x
            current_y = self.pose.pose.pose.position.y
            bearing2target_inertial = current_yaw + msg.targets[i].bearing_rad
            projected_target_x = msg.targets[i].range_m * np.cos( bearing2target_inertial ) + current_x
            projected_target_y = msg.targets[i].range_m * np.sin( bearing2target_inertial ) + current_y

            # Attempt to associate with a landmark
            for l in self.landmark_dict.keys():
                landmark_x, landmark_y, landmark_z = self.landmark_dict[l]
                dist = np.linalg.norm([projected_target_x - landmark_x, projected_target_y - landmark_y])
                if dist < self.association_threshold_distance:
                    msg.targets[i].id = "landmark_" + l
                    self.cuprint("associating detection with: " + l)
                    break
        
        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("sonar_association")
    d = SonarAssociator()
    rospy.spin()
