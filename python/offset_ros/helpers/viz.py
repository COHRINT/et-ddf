#!/usr/bin/env python

"""
Performance visualization for applications of ET-DDF.
"""

import rospy
import numpy as np

from offset_ros.helpers.msg_conversion import inflate_covariance

from offset_etddf.msg import AgentState, AgentMeasurement
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class PerformanceViz(object):
    """
    ROS node class to take in local estimates from agents, as well as ground
    truth data, and visualize ownship estimate error during experiments.

    Publishes ownship estimate error topics to be visulized with rqt_plot.
    """

    def __init__(self):
        

        # initialize ros node
        rospy.init_node('performance_viz')

        # create subscribers to all estimate and ground truth topics
        rospy.Subscriber('/agent_0/local_estimate',AgentState,self.estimate_cb_0)
        rospy.Subscriber('/agent_1/local_estimate',AgentState,self.estimate_cb_1)
        rospy.Subscriber('/agent_2/local_estimate',AgentState,self.estimate_cb_2)
        rospy.Subscriber('/agent_3/local_estimate',AgentState,self.estimate_cb_3)

        rospy.Subscriber('/agent_0/pose_gt',Odometry,self.gt_cb_0)
        rospy.Subscriber('/agent_1/pose_gt',Odometry,self.gt_cb_1)
        rospy.Subscriber('/agent_2/pose_gt',Odometry,self.gt_cb_2)
        rospy.Subscriber('/agent_3/pose_gt',Odometry,self.gt_cb_3)

        # create publishers for estimate error
        self.est_err_pub_0 = rospy.Publisher('/agent_0/estimate_error',Point,queue_size=10)
        self.est_err_pub_1 = rospy.Publisher('/agent_1/estimate_error',Point,queue_size=10)
        self.est_err_pub_2 = rospy.Publisher('/agent_2/estimate_error',Point,queue_size=10)
        self.est_err_pub_3 = rospy.Publisher('/agent_3/estimate_error',Point,queue_size=10)

        # create publishers for estimate error
        self.est_cov_pub_0 = rospy.Publisher('/agent_0/estimate_covariance',Point,queue_size=10)
        self.est_cov_pub_1 = rospy.Publisher('/agent_1/estimate_covariance',Point,queue_size=10)
        self.est_cov_pub_2 = rospy.Publisher('/agent_2/estimate_covariance',Point,queue_size=10)
        self.est_cov_pub_3 = rospy.Publisher('/agent_3/estimate_covariance',Point,queue_size=10)

        # create MSE publishers
        self.mse_pub_0 = rospy.Publisher('/agent_0/position_mse',Float64,queue_size=10)
        self.mse_pub_1 = rospy.Publisher('/agent_1/position_mse',Float64,queue_size=10)
        self.mse_pub_2 = rospy.Publisher('/agent_2/position_mse',Float64,queue_size=10)
        self.mse_pub_3 = rospy.Publisher('/agent_3/position_mse',Float64,queue_size=10)

        self.recent_ground_truth_0 = [0,0]
        self.recent_ground_truth_1 = [0,0]
        self.recent_ground_truth_2 = [0,0]
        self.recent_ground_truth_3 = [0,0]

        # wait for messages
        rospy.spin()

    def estimate_cb_0(self,msg):
        """
        Compute estimate error from most recent estimate and ground truth messages,
        published to estimate error topic.
        """

        new_msg = Point()
        new_msg.x = msg.mean[0] - self.recent_ground_truth_0[0]
        new_msg.y = msg.mean[2] - self.recent_ground_truth_0[1]

        cov_msg = Point()
        cov = inflate_covariance(msg.covariance)
        cov_msg.x = 2*np.sqrt(cov[0,0])
        cov_msg.y = 2*np.sqrt(cov[2,2])

        mse_msg = Float64()
        mse_msg.data = float(np.linalg.norm(np.array( [msg.mean[0],msg.mean[2]] )- np.array(self.recent_ground_truth_0)))
        
        self.est_err_pub_0.publish(new_msg)
        self.est_cov_pub_0.publish(cov_msg)
        self.mse_pub_0.publish(mse_msg)

    def estimate_cb_1(self,msg):
        """
        Compute estimate error from most recent estimate and ground truth messages,
        published to estimate error topic.
        """

        new_msg = Point()
        new_msg.x = msg.mean[4] - self.recent_ground_truth_1[0]
        new_msg.y = msg.mean[6] - self.recent_ground_truth_1[1]

        cov_msg = Point()
        cov = inflate_covariance(msg.covariance)
        cov_msg.x = 2*np.sqrt(cov[4,4])
        cov_msg.y = 2*np.sqrt(cov[6,6])

        mse_msg = Float64()
        mse_msg.data = float(np.linalg.norm(np.array( [msg.mean[4],msg.mean[6]] )- np.array(self.recent_ground_truth_1)))
        
        self.est_err_pub_1.publish(new_msg)
        self.est_cov_pub_1.publish(cov_msg)
        self.mse_pub_1.publish(mse_msg)

    def estimate_cb_2(self,msg):
        """
        Compute estimate error from most recent estimate and ground truth messages,
        published to estimate error topic.
        """

        new_msg = Point()
        new_msg.x = msg.mean[0] - self.recent_ground_truth_2[0]
        new_msg.y = msg.mean[2] - self.recent_ground_truth_2[1]

        cov_msg = Point()
        cov = inflate_covariance(msg.covariance)
        cov_msg.x = 2*np.sqrt(cov[8,8])
        cov_msg.y = 2*np.sqrt(cov[10,10])

        mse_msg = Float64()
        mse_msg.data = float(np.linalg.norm(np.array( [msg.mean[8],msg.mean[10]] )- np.array(self.recent_ground_truth_2)))
        
        self.est_err_pub_2.publish(new_msg)
        self.est_cov_pub_2.publish(cov_msg)
        self.mse_pub_2.publish(mse_msg)

    def estimate_cb_3(self,msg):
        """
        Compute estimate error from most recent estimate and ground truth messages,
        published to estimate error topic.
        """

        new_msg = Point()
        new_msg.x = msg.mean[0] - self.recent_ground_truth_3[0]
        new_msg.y = msg.mean[2] - self.recent_ground_truth_3[1]

        cov_msg = Point()
        cov = inflate_covariance(msg.covariance)
        cov_msg.x = 2*np.sqrt(cov[12,12])
        cov_msg.y = 2*np.sqrt(cov[14,14])

        mse_msg = Float64()
        mse_msg.data = float(np.linalg.norm(np.array( [msg.mean[12],msg.mean[14]] )- np.array(self.recent_ground_truth_3)))
        
        self.est_err_pub_3.publish(new_msg)
        self.est_cov_pub_3.publish(cov_msg)
        self.mse_pub_3.publish(mse_msg)

    def gt_cb_0(self,msg):
        """
        Saves ground truth information.
        """
        self.recent_ground_truth_0[0] = msg.pose.pose.position.x
        self.recent_ground_truth_0[1] = msg.pose.pose.position.y

    def gt_cb_1(self,msg):
        """
        Saves ground truth information.
        """
        self.recent_ground_truth_1[0] = msg.pose.pose.position.x
        self.recent_ground_truth_1[1] = msg.pose.pose.position.y

    def gt_cb_2(self,msg):
        """
        Saves ground truth information.
        """
        self.recent_ground_truth_2[0] = msg.pose.pose.position.x
        self.recent_ground_truth_2[1] = msg.pose.pose.position.y

    def gt_cb_3(self,msg):
        """
        Saves ground truth information.
        """
        self.recent_ground_truth_3[0] = msg.pose.pose.position.x
        self.recent_ground_truth_3[1] = msg.pose.pose.position.y





if __name__ == "__main__":
    PerformanceViz()

