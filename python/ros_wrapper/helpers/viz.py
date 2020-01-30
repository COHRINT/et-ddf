#!/usr/bin/env python

"""
Performance visualization for applications of ET-DDF.
"""

import rospy
import copy
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
                [0.0,0.5,0.5],
                [0.5,0.5,0.0],
                [0.33,0.33,0.33]]

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
        agent_name = 'bluerov2'
        self.ordered_ids = []
        for conn in self.connections:
            self.ordered_ids += conn
        self.ordered_ids = sorted(list(set(self.ordered_ids)))

        self.est_err_pubs = []
        self.est_cov_pubs = []
        self.rmse_pubs = []
        self.recent_gts = [[0,0,0] for id_ in self.ordered_ids]

        for id_ in self.ordered_ids:
            # create subscribers to et-ddf estimates and ground truth
            rospy.Subscriber('/{}_{}/local_estimate'.format(agent_name,id_), AgentState, self.estimate_cb, callback_args=id_)
            rospy.Subscriber('/{}_{}/local_estimate'.format(agent_name,id_), AgentState, self.marker_pub_fxn, callback_args=id_)
            rospy.Subscriber('/{}_{}/pose_gt'.format(agent_name,id_), Odometry, self.gt_cb, callback_args=id_)

            # create publishers for estimate error, covariance, and rmse
            self.est_err_pubs.append(rospy.Publisher('/{}_{}/estimate_error'.format(agent_name,id_), Point, queue_size=10))
            self.est_cov_pubs.append(rospy.Publisher('/{}_{}/estimate_covariance'.format(agent_name,id_), Point, queue_size=10))
            self.rmse_pubs.append(rospy.Publisher('/{}_{}/position_rmse'.format(agent_name,id_), Float64, queue_size=10))

        # rviz marker publisher
        self.marker_pub = rospy.Publisher('visualization_marker',Marker,queue_size=10)

        # wait for messages
        rospy.spin()

    def estimate_cb(self,msg,id_):
        """
        Compute estimate error from most recent estimate and ground truth messages, publshed to estimate error topics
        """
        # get source id
        agent_id = msg.src
        id_index = self.ordered_ids.index(agent_id)
        conn_ids = sorted(copy.deepcopy(self.connections[id_index]) + [agent_id])
        
        # build list of distance one and distance two neighbors for each agent
        # each agent gets full list of connections
        neighbor_conn_ids = []
        for j in range(0,len(self.connections[id_index])):
            for k in range(0,len(self.connections[self.ordered_ids.index(self.connections[id_index][j])])):

                if not self.connections[self.ordered_ids.index(self.connections[id_index][j])][k] in neighbor_conn_ids:
                    neighbor_conn_ids += self.connections[self.ordered_ids.index(self.connections[id_index][j])]

                # remove agent's own id from list of neighbors
                if agent_id in neighbor_conn_ids:
                    neighbor_conn_ids.remove(agent_id)

        # combine with direct connection ids and sort
        conn_ids = list(set(sorted(conn_ids + neighbor_conn_ids)))

        # divide out direct measurement connections and all connections
        connections_new = list(set(sorted(neighbor_conn_ids + self.connections[id_index])))

        # find ownship location in ids
        ownship_index = conn_ids.index(agent_id)

        new_msg = Point()
        new_msg.x = msg.mean[6*ownship_index] - self.recent_gts[self.ordered_ids.index(agent_id)][0]
        new_msg.y = msg.mean[6*ownship_index+2] - self.recent_gts[self.ordered_ids.index(agent_id)][1]
        new_msg.z = msg.mean[6*ownship_index+4] - self.recent_gts[self.ordered_ids.index(agent_id)][2]

        cov_msg = Point()
        cov = inflate_covariance(msg.covariance)
        cov_msg.x = 2*np.sqrt(cov[6*ownship_index,6*ownship_index])
        cov_msg.y = 2*np.sqrt(cov[6*ownship_index+2,6*ownship_index+2])
        cov_msg.z = 2*np.sqrt(cov[6*ownship_index+4,6*ownship_index+4])

        rmse_msg = Float64()
        rmse_msg.data = float(np.linalg.norm(np.array( [msg.mean[6*ownship_index],msg.mean[6*ownship_index+2],msg.mean[6*ownship_index+4]] )- np.array(self.recent_gts[self.ordered_ids.index(agent_id)])))
        
        self.est_err_pubs[self.ordered_ids.index(agent_id)].publish(new_msg)
        self.est_cov_pubs[self.ordered_ids.index(agent_id)].publish(cov_msg)
        self.rmse_pubs[self.ordered_ids.index(agent_id)].publish(rmse_msg)

        #self.marker_pub_fxn(msg)

    def gt_cb(self,msg,id_):
        """
        Record vehicle ground truth for estimate error computation.
        """
        # get id of vehicle from publisher name
        agent_id = id_
        
        # record
        self.recent_gts[self.ordered_ids.index(agent_id)][0] = msg.pose.pose.position.x
        self.recent_gts[self.ordered_ids.index(agent_id)][1] = msg.pose.pose.position.y
        self.recent_gts[self.ordered_ids.index(agent_id)][2] = msg.pose.pose.position.z

    def marker_pub_fxn(self,msg,id_):
        """
        Publish rviz markers for estimate means and covariances.
        """
        # extract the mean and covariance from the incoming message
        # get ownship location in estimate
        agent_id = msg.src
        id_index = self.ordered_ids.index(agent_id)
        conn_ids = sorted(copy.deepcopy(self.connections[id_index]) + [agent_id])

        # build list of distance one and distance two neighbors for each agent
        # each agent gets full list of connections
        neighbor_conn_ids = []
        for j in range(0,len(self.connections[id_index])):
            for k in range(0,len(self.connections[self.ordered_ids.index(self.connections[id_index][j])])):

                if not self.connections[self.ordered_ids.index(self.connections[id_index][j])][k] in neighbor_conn_ids:
                    neighbor_conn_ids += self.connections[self.ordered_ids.index(self.connections[id_index][j])]

                # remove agent's own id from list of neighbors
                if agent_id in neighbor_conn_ids:
                    neighbor_conn_ids.remove(agent_id)

        # combine with direct connection ids and sort
        conn_ids = list(set(sorted(conn_ids + neighbor_conn_ids)))

        # divide out direct measurement connections and all connections
        connections_new = list(set(sorted(neighbor_conn_ids + self.connections[id_index])))

        # find ownship location in ids
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
    # sort fromn largest to smallest
    idx = l.argsort()[::-1]
    l = l[idx]
    v = v[:,idx]

    conf_level = chi2.isf(1-alpha,3)

    x_scale = 2*np.sqrt(conf_level*l[0])
    y_scale = 2*np.sqrt(conf_level*l[1])
    z_scale = 2*np.sqrt(conf_level*l[2])

    # compute quaternion rotation from eigenvectors matrix --> uses [w,x,y,z] quaternion notation
    q0 = 0.5*np.sqrt(1 + v[0,0]+ v[1,1] + v[2,2])
    q1 = (v[2,1]-v[1,2])/4*q0
    q2 = (v[0,2]-v[2,0])/4*q0
    q3 = (v[1,0]-v[0,1])/4*q0

    # normalize quaternion
    qnorm = np.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
    q0 /= qnorm
    q1 /= qnorm
    q2 /= qnorm
    q3 /= qnorm

    return np.array([x_scale,y_scale,z_scale]), np.array([q0,q1,q2,q3])

def test_marker_viz():
    n = PerformanceViz()
    n.marker_pub_fxn('')


if __name__ == "__main__":
    PerformanceViz()
    # test_marker_viz()

