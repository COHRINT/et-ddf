#!/usr/bin/env python

"""
Performance visualization for applications of ET-DDF.
"""

import rospy
import numpy as np
from scipy.stats import chi2

from ros_wrapper.helpers.msg_conversion import inflate_covariance

from etddf_ros.msg import AgentState, AgentMeasurement
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

MARKER_COLOR = [[1.0,0.0,0.0],
                [0.0,1.0,0.0],
                [0.0,0.0,1.0],
                [0.0,0.5,0.5]]

class PerformanceViz(object):
    """
    ROS node class to take in local estimates from agents, as well as ground
    truth data, and visualize ownship estimate error during experiments.

    Publishes ownship estimate error topics to be visulized with rqt_plot.
    """

    def __init__(self):
        

        # initialize ros node
        rospy.init_node('performance_viz')

        # get parameters from parameter server
        self.connections = rospy.get_param('/bluerov2_3/connections')
        self.ordered_ids = []
        for conn in self.connections:
            self.ordered_ids += conn
        self.ordered_ids = sorted(list(set(self.ordered_ids)))

        # create subscribers to all estimate and ground truth topics
        rospy.Subscriber('/bluerov2_3/local_estimate',AgentState,self.estimate_cb_0)
        rospy.Subscriber('/bluerov2_4/local_estimate',AgentState,self.estimate_cb_1)
        rospy.Subscriber('/bluerov2_5/local_estimate',AgentState,self.estimate_cb_2)
        rospy.Subscriber('/bluerov2_6/local_estimate',AgentState,self.estimate_cb_3)

        rospy.Subscriber('/bluerov2_3/pose_gt',Odometry,self.gt_cb_0)
        rospy.Subscriber('/bluerov2_4/pose_gt',Odometry,self.gt_cb_1)
        rospy.Subscriber('/bluerov2_5/pose_gt',Odometry,self.gt_cb_2)
        rospy.Subscriber('/bluerov2_6/pose_gt',Odometry,self.gt_cb_3)

        # create publishers for estimate error
        self.est_err_pub_0 = rospy.Publisher('/bluerov2_3/estimate_error',Point,queue_size=10)
        self.est_err_pub_1 = rospy.Publisher('/bluerov2_4/estimate_error',Point,queue_size=10)
        self.est_err_pub_2 = rospy.Publisher('/bluerov2_5/estimate_error',Point,queue_size=10)
        self.est_err_pub_3 = rospy.Publisher('/bluerov2_6/estimate_error',Point,queue_size=10)

        # create publishers for estimate error
        self.est_cov_pub_0 = rospy.Publisher('/bluerov2_3/estimate_covariance',Point,queue_size=10)
        self.est_cov_pub_1 = rospy.Publisher('/bluerov2_4/estimate_covariance',Point,queue_size=10)
        self.est_cov_pub_2 = rospy.Publisher('/bluerov2_5/estimate_covariance',Point,queue_size=10)
        self.est_cov_pub_3 = rospy.Publisher('/bluerov2_6/estimate_covariance',Point,queue_size=10)

        # create MSE publishers
        self.mse_pub_0 = rospy.Publisher('/bluerov2_3/position_mse',Float64,queue_size=10)
        self.mse_pub_1 = rospy.Publisher('/bluerov2_4/position_mse',Float64,queue_size=10)
        self.mse_pub_2 = rospy.Publisher('/bluerov2_5/position_mse',Float64,queue_size=10)
        self.mse_pub_3 = rospy.Publisher('/bluerov2_6/position_mse',Float64,queue_size=10)

        self.recent_ground_truth_0 = [0,0,0]
        self.recent_ground_truth_1 = [0,0,0]
        self.recent_ground_truth_2 = [0,0,0]
        self.recent_ground_truth_3 = [0,0,0]

        # rviz marker publisher
        self.marker_pub = rospy.Publisher('visualization_marker',Marker,queue_size=10)

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
        new_msg.z = msg.mean[4] - self.recent_ground_truth_0[2]

        cov_msg = Point()
        cov = inflate_covariance(msg.covariance)
        cov_msg.x = 2*np.sqrt(cov[0,0])
        cov_msg.y = 2*np.sqrt(cov[2,2])
        cov_msg.z = 2*np.sqrt(cov[4,4])

        mse_msg = Float64()
        mse_msg.data = float(np.linalg.norm(np.array( [msg.mean[0],msg.mean[2],msg.mean[4]] )- np.array(self.recent_ground_truth_0)))
        
        self.est_err_pub_0.publish(new_msg)
        self.est_cov_pub_0.publish(cov_msg)
        self.mse_pub_0.publish(mse_msg)

        self.marker_pub_fxn(msg)

    def estimate_cb_1(self,msg):
        """
        Compute estimate error from most recent estimate and ground truth messages,
        published to estimate error topic.
        """

        new_msg = Point()
        new_msg.x = msg.mean[6] - self.recent_ground_truth_1[0]
        new_msg.y = msg.mean[8] - self.recent_ground_truth_1[1]
        new_msg.z = msg.mean[10] - self.recent_ground_truth_1[2]

        cov_msg = Point()
        cov = inflate_covariance(msg.covariance)
        cov_msg.x = 2*np.sqrt(cov[6,6])
        cov_msg.y = 2*np.sqrt(cov[8,8])
        cov_msg.z = 2*np.sqrt(cov[10,10])

        mse_msg = Float64()
        mse_msg.data = float(np.linalg.norm(np.array( [msg.mean[6],msg.mean[8],msg.mean[10]] )- np.array(self.recent_ground_truth_1)))
        
        self.est_err_pub_1.publish(new_msg)
        self.est_cov_pub_1.publish(cov_msg)
        self.mse_pub_1.publish(mse_msg)

        self.marker_pub_fxn(msg)

    def estimate_cb_2(self,msg):
        """
        Compute estimate error from most recent estimate and ground truth messages,
        published to estimate error topic.
        """

        new_msg = Point()
        new_msg.x = msg.mean[12] - self.recent_ground_truth_2[0]
        new_msg.y = msg.mean[14] - self.recent_ground_truth_2[1]
        new_msg.z = msg.mean[16] - self.recent_ground_truth_2[2]

        cov_msg = Point()
        cov = inflate_covariance(msg.covariance)
        cov_msg.x = 2*np.sqrt(cov[12,12])
        cov_msg.y = 2*np.sqrt(cov[14,14])
        cov_msg.z = 2*np.sqrt(cov[16,16])

        mse_msg = Float64()
        mse_msg.data = float(np.linalg.norm(np.array( [msg.mean[12],msg.mean[14],msg.mean[16]] )- np.array(self.recent_ground_truth_2)))
        
        self.est_err_pub_2.publish(new_msg)
        self.est_cov_pub_2.publish(cov_msg)
        self.mse_pub_2.publish(mse_msg)

        self.marker_pub_fxn(msg)

    def estimate_cb_3(self,msg):
        """
        Compute estimate error from most recent estimate and ground truth messages,
        published to estimate error topic.
        """

        new_msg = Point()
        new_msg.x = msg.mean[12] - self.recent_ground_truth_3[0]
        new_msg.y = msg.mean[14] - self.recent_ground_truth_3[1]
        new_msg.y = msg.mean[16] - self.recent_ground_truth_3[2]

        cov_msg = Point()
        cov = inflate_covariance(msg.covariance)
        cov_msg.x = 2*np.sqrt(cov[12,12])
        cov_msg.y = 2*np.sqrt(cov[14,14])
        cov_msg.z = 2*np.sqrt(cov[16,16])

        mse_msg = Float64()
        mse_msg.data = float(np.linalg.norm(np.array( [msg.mean[12],msg.mean[14],msg.mean[16]] )- np.array(self.recent_ground_truth_3)))
        
        self.est_err_pub_3.publish(new_msg)
        self.est_cov_pub_3.publish(cov_msg)
        self.mse_pub_3.publish(mse_msg)

        self.marker_pub_fxn(msg)

    def gt_cb_0(self,msg):
        """
        Saves ground truth information.
        """
        self.recent_ground_truth_0[0] = msg.pose.pose.position.x
        self.recent_ground_truth_0[1] = msg.pose.pose.position.y
        self.recent_ground_truth_0[2] = msg.pose.pose.position.z

    def gt_cb_1(self,msg):
        """
        Saves ground truth information.
        """
        self.recent_ground_truth_1[0] = msg.pose.pose.position.x
        self.recent_ground_truth_1[1] = msg.pose.pose.position.y
        self.recent_ground_truth_1[2] = msg.pose.pose.position.z

    def gt_cb_2(self,msg):
        """
        Saves ground truth information.
        """
        self.recent_ground_truth_2[0] = msg.pose.pose.position.x
        self.recent_ground_truth_2[1] = msg.pose.pose.position.y
        self.recent_ground_truth_2[2] = msg.pose.pose.position.z

    def gt_cb_3(self,msg):
        """
        Saves ground truth information.
        """
        self.recent_ground_truth_3[0] = msg.pose.pose.position.x
        self.recent_ground_truth_3[1] = msg.pose.pose.position.y
        self.recent_ground_truth_3[2] = msg.pose.pose.position.z

    def marker_pub_fxn(self,msg):
        """
        Publish rviz markers for estimate means and covariances.
        """
        # extract the mean and covariance from the incoming message
        # get ownship location in estimate
        agent_id = msg.src
        id_index = self.ordered_ids.index(agent_id)
        conn_ids = sorted(self.connections[id_index] + [agent_id])
        ownship_index = conn_ids.index(agent_id)

        estimate_mean = np.take(msg.mean[6*ownship_index:6*ownship_index+6],[0,2,4])
        estimate_covariance = inflate_covariance(msg.covariance)[6*ownship_index:6*ownship_index+6][np.ix_([0,2,4],[0,2,4])]

        # compute covariance ellipse scaling and rotation (quaternion comes out as [w,x,y,z])
        scale, rotation_quat = uncertain_ellipse(estimate_covariance)

        marker_color_list = MARKER_COLOR[id_index]

        new_marker = Marker()
        new_marker.header.frame_id = "map"
        new_marker.header.stamp = rospy.Time()
        new_marker.id = msg.src
        new_marker.type = Marker.SPHERE # SPHERE
        new_marker.action = Marker.ADD # MODIFY
        new_marker.pose.position.x = estimate_mean[0]
        new_marker.pose.position.y = estimate_mean[1]
        new_marker.pose.position.z = estimate_mean[2]
        new_marker.pose.orientation.x = rotation_quat[1]
        new_marker.pose.orientation.y = rotation_quat[2]
        new_marker.pose.orientation.z = rotation_quat[3]
        new_marker.pose.orientation.w = rotation_quat[0]
        new_marker.scale.x = scale[0]
        new_marker.scale.y = scale[1]
        new_marker.scale.z = scale[2]
        new_marker.color.a = 0.5
        new_marker.color.r = marker_color_list[0]
        new_marker.color.g = marker_color_list[1]
        new_marker.color.b = marker_color_list[2]
        new_marker.lifetime = rospy.Duration(100)

        self.marker_pub.publish(new_marker)

def uncertain_ellipse(covariance,alpha=0.95):
    """
    Compute semi-major and semi-minor axis dimensions, and rotation.
    Alpha defines confidence level for ellipse.
    Assumes 3x3 covariance.
    """
    # compute eigenvalues and vectors of covariance
    l, v = np.linalg.eig(covariance)

    conf_level = chi2.isf(1-alpha,3)

    x_scale = 2*np.sqrt(conf_level*l[0])
    y_scale = 2*np.sqrt(conf_level*l[1])
    z_scale = 2*np.sqrt(conf_level*l[2])

    # compute quaternion rotation from eigenvectors matrix --> uses [w,x,y,z] quaternion notation
    q0 = 0.5*np.sqrt(1 + v[0,0]+ v[1,1] + v[2,2])
    q1 = (v[2,1]-v[1,2])/4*q0
    q2 = (v[0,2]-v[2,0])/4*q0
    q3 = (v[1,0]-v[0,1])/4*q0

    return np.array([x_scale,y_scale,z_scale]), np.array([q0,q1,q2,q3])

def test_marker_viz():
    n = PerformanceViz()
    n.marker_pub_fxn('')


if __name__ == "__main__":
    PerformanceViz()
    # test_marker_viz()

