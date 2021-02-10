#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from minau.msg import SonarTargetList, SonarTarget
import tf
import numpy as np
from cuprint.cuprint import CUPrint

class SonarAssociator:

    def __init__(self):
        
        self.cuprint = CUPrint("SonarAssociator")
        self.cuprint("Loading")
        self.association_threshold_distance = rospy.get_param("~threshold",1)

        self.landmark_dict = rospy.get_param("~landmarks", {})

        # pose_topic = "etddf/estimate" + rospy.get_namespace()[:-1]
        pose_topic = rospy.get_namespace()[:-1] + "/pose_gt"
        rospy.Subscriber(pose_topic, Odometry, self.pose_callback)
        rospy.wait_for_message(pose_topic, Odometry)

        self.red_team_names = rospy.get_param("~red_team_names")
        blue_team = rospy.get_param("~blue_team_names")
        self.blue_team = {}
        for b in blue_team:
            if b == "surface":
                continue
            rospy.Subscriber("etddf/estimate/" + b, Odometry, self.blue_team_callback, callback_args=b)
            self.cuprint("Waiting for msg: " + "etddf/estimate/" + b)
            rospy.wait_for_message("etddf/estimate/" + b, Odometry)

        self.pub = rospy.Publisher("sonar_processing/target_list/associated", SonarTargetList, queue_size=10)

        sonar_topic = "sonar_processing/target_list"
        rospy.Subscriber(sonar_topic, SonarTargetList, self.sonar_callback)

        self.cuprint("Loaded")

    def pose_callback(self, msg):
        self.pose = msg.pose

    def blue_team_callback(self, msg, agent_name):
        self.blue_team[agent_name] = msg.pose

    def sonar_callback(self, msg):
        for i in range(len(msg.targets)):

            # Debugging
            original_id = msg.targets[i].id


            ori = self.pose.pose.orientation
            _, _, current_yaw = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            current_x = self.pose.pose.position.x
            current_y = self.pose.pose.position.y

            bearing2target_inertial = current_yaw + msg.targets[i].bearing_rad

            projected_target_x = msg.targets[i].range_m * np.cos( bearing2target_inertial ) + current_x
            projected_target_y = msg.targets[i].range_m * np.sin( bearing2target_inertial ) + current_y

            # Attempt to associate with a landmark
            associated = False
            dists = []
            # for l in self.landmark_dict.keys():
            #     landmark_x, landmark_y, landmark_z = self.landmark_dict[l]
            #     dist = np.linalg.norm([projected_target_x - landmark_x, projected_target_y - landmark_y])
            #     if dist < self.association_threshold_distance:
            #         msg.targets[i].id = "landmark_" + l
            #         self.cuprint(l)
            #         info = [x*(180 / np.pi) for x in [current_yaw, msg.targets[i].bearing_rad, bearing2target_inertial]]
            #         # print("Vehicle Orientation, Measured Bearing, Estimated World Bearing: " + str(info))
            #         associated = True
            #         break
            #     else:
            #         dists.append(dist)

            # Attempt to associate with a blue team member
            for b in self.blue_team.keys():
                blue_position = self.blue_team[b].pose.position
                blue_x, blue_y, blue_z = blue_position.x, blue_position.y, blue_position.z
                dist = np.linalg.norm([projected_target_x - blue_x, projected_target_y - blue_y])
                if dist < self.association_threshold_distance:
                    msg.targets[i].id = b
                    # self.cuprint(b)
                    info = [x*(180 / np.pi) for x in [current_yaw, msg.targets[i].bearing_rad, bearing2target_inertial]]
                    # print("Vehicle Orientation, Measured Bearing, Estimated World Bearing: " + str(info))
                    associated = True
                    break
                else:
                    dists.append(dist)
            
            # If not association --> assume it's the red asset
            # TODO future logic 
            if not associated and self.red_team_names:
                # self.cuprint("Associating detection with red agent")
                msg.targets[i].id = self.red_team_names[0]

            if msg.targets[i].id != original_id:
                
                print(msg.targets[i])
                blue_team = []
                for b in self.blue_team.keys():
                    blue_position = self.blue_team[b].pose.position
                    blue_team.append( [blue_position.x, blue_position.y, blue_position.z] )
                self.cuprint("MISSACCOCIATION!", warn=True)
                print(self.blue_team)
                print(msg.targets[i].bearing_rad)
                print(msg.targets[i].range_m)
            # else:
            #     self.cuprint("Successful association with: {}".format(msg.targets[i].id))
            # print("Norms: " + str(dists) + " for " + str(self.landmark_dict.keys()))
        
        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("sonar_association")
    d = SonarAssociator()
    rospy.spin()
