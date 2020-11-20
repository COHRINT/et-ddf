#!/usr/bin/env python
from __future__ import division

"""
Implementation of an aided strapdown inertial navigation system.

"""
import rospy
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovariance, Vector3, Twist, TwistWithCovariance
import numpy as np
from numpy.linalg import inv
from numpy import dot
np.set_printoptions(suppress=True, precision=2)
from scipy.linalg import block_diag, sqrtm
import tf
import threading
import sys
from etddf.msg import PositionVelocity
from cuprint.cuprint import CUPrint

__author__ = "Luke Barbier"
__copyright__ = "Copyright 2020, COHRINT Lab"
__email__ = "luke.barbier@colorado.edu"
__status__ = "Development"
__license__ = "MIT"
__maintainer__ = "Luke Barbier"
__version__ = "2.0"

G_ACCEL = 9.80665 # gravitational acceleration [m/s/s]
NUM_STATES = 16

class StrapdownINS:
    """
    Implementation of an aided strapdown intertial navigation filter.  
    State: [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, 
            q_0, q_1, q_2, q_3, b_ax, b_ay, b_az, b_wx, b_wy, b_wz]

    Parameters
    ----------

    """
    def __init__(self, x0, P0, Q, default_meas_variance, strapdown=True):
        self.cuprint = CUPrint(rospy.get_name())

        self.last_update_time = None
        self.update_lock = threading.Lock()
        # initialize estimate and covariance
        self.x = x0.reshape(-1)
        self.P = P0
        self.Q = Q
        self.pub = rospy.Publisher("strapdown/estimate", Odometry, queue_size=10)
        self.ci_pub = rospy.Publisher("strapdown/intersection", PositionVelocity, queue_size=1)
        self.last_depth = None
        self.last_intersection = None
        self.data_x, self.data_y = None, None
        self.dvl_x, self.dvl_y = None, None
        self.skip_multiplexer = 0
        rospy.sleep(rospy.get_param("~wait_on_startup"))
        # rospy.Subscriber("mavros/global_position/rel_alt", Float64, self.depth_callback, queue_size=1)
        rospy.Subscriber("baro", Float64, self.depth_callback, queue_size=1)
        rospy.Subscriber("dvl", Vector3, self.dvl_callback, queue_size=1)
        # rospy.Subscriber("pose_gt", Odometry, self.gps_callback, queue_size=1)

        if strapdown:
            rospy.Subscriber("imu", Imu, self.propagate_strap, queue_size=1)
        else:
            rospy.Subscriber("imu", Imu, self.propagate_normal, queue_size=1)
        rospy.Subscriber("strapdown/intersection_result", PositionVelocity, self.intersection_result)
        self.cuprint("loaded")

    def publish_intersection(self):
        trans_strap2ci = np.zeros((8,8))
        trans_strap2ci[:3,:3] = np.eye(3)
        trans_strap2ci[3:6, 4:7] = np.eye(3)
        trans_strap2ci[6,3] = 1
        trans_strap2ci[7,7] = 1

        x = dot(trans_strap2ci, self.x.reshape(-1,1))
        P = dot(trans_strap2ci, self.P).dot(trans_strap2ci.T)

        position = Vector3(x[0], x[1], x[2])
        velocity = Vector3(x[3], x[4], x[5])
        cov = list(P[:6,:6].flatten())
        msg = PositionVelocity(position, velocity, cov)
        self.ci_pub.publish(msg)

    def publish_intersection_strapdown(self):
        position = Vector3(self.x[0], self.x[1], self.x[2])
        velocity = Vector3(self.x[3], self.x[4], self.x[5])
        cov = list(self.P[:6,:6].flatten())
        msg = PositionVelocity(position, velocity, cov)
        self.ci_pub.publish(msg)

    def intersection_result(self, msg):
        self.last_intersection = msg

    def dvl_callback(self, msg):
        theta = self.x[3]
        theta2 = theta - np.pi / 2.0
        body2inertial = np.array([[np.cos(theta), np.cos(theta2)],[np.sin(theta), np.sin(theta2)]])
        vel_inertial = np.dot(body2inertial, np.array([[msg.x],[msg.y]]))
        self.dvl_x = vel_inertial[0,0]
        self.dvl_y = vel_inertial[1,0]

    def gps_callback(self, msg):
        # if self.skip_multiplexer % 300 == 0:
        self.data_x = msg.pose.pose.position.x + np.random.normal(0, scale=0.05)
        self.data_y = msg.pose.pose.position.y + np.random.normal(0, scale=0.05)

    def depth_callback(self, msg):
        self.last_depth = msg.data

    def publish_estimate(self, update_seq, timestamp):
        quat = tf.transformations.quaternion_from_euler(0, 0, self.x[3])
        pose = Pose(Point(self.x[0], self.x[1], self.x[2]), \
            Quaternion(quat[0], quat[1], quat[2], quat[3]))
        pose_cov = np.zeros((6,6))
        pose_cov[:3,:3] = self.P[:3,:3]
        pose_cov[5,5] = self.P[3,3]
        pwc = PoseWithCovariance(pose, list(pose_cov.flatten()))

        tw = Twist(Vector3(self.x[4], self.x[5], self.x[6]), \
            Vector3(0, 0, self.x[7]))
        twist_cov = np.zeros((6,6))
        twist_cov[:3,:3] = self.P[4:7, 4:7]
        twist_cov[5,5] = self.P[7,7]
        twc = TwistWithCovariance(tw, list(twist_cov.flatten()))
        h = Header(update_seq, timestamp, "map")
        o = Odometry(h, "map", pwc, twc)
        self.pub.publish(o)

    def publish_estimate_strapdown(self, update_seq, timestamp, angular_velocity, angular_velocity_cov):
        pose = Pose(Point(self.x[0], self.x[1], self.x[2]), \
            Quaternion(self.x[6], self.x[7], self.x[8], self.x[9]))
        pose_cov = np.zeros((6,6))
        pose_cov[:3,:3] = self.P[:3,:3]
        pwc = PoseWithCovariance(pose, list(pose_cov.flatten()))
        tw = Twist(Vector3(self.x[3], self.x[4], self.x[5]), \
            Vector3(angular_velocity[0], angular_velocity[1], angular_velocity[2]))
        twist_cov = np.zeros((6,6))
        twist_cov[:3,:3] = self.P[3:6, 3:6]
        twist_cov[3:, 3:] = angular_velocity_cov
        twc = TwistWithCovariance(tw, list(twist_cov.flatten()))
        h = Header(update_seq, timestamp, "map")
        o = Odometry(h, "map", pwc, twc)
        self.pub.publish(o)

    def normalize_angle_states(self):
        self.x[3] = self.normalize_angle(self.x[3])
        
    def normalize_angle(self, angle):
        return np.mod( angle + np.pi, 2*np.pi) - np.pi

    def propagate_normal(self, imu_measurement):
        if self.last_update_time is None:
            self.last_update_time = imu_measurement.header.stamp

            # Instantiate the filter at the first compass meas
            _, _, yaw = tf.transformations.euler_from_quaternion([imu_measurement.orientation.x,\
                                                                imu_measurement.orientation.y, \
                                                                imu_measurement.orientation.z,\
                                                                imu_measurement.orientation.w])
            self.x[3] = yaw
            self.P[3,3] = 0.1
            return
        else:
            delta_t_ros =  imu_measurement.header.stamp - self.last_update_time
            self.last_update_time = imu_measurement.header.stamp
            dt = delta_t_ros.to_sec()
        self.update_lock.acquire()

        x = self.x.reshape(-1,1)

        A = np.eye(NUM_STATES)
        num_base_states = 4
        for d in range(num_base_states):
            A[d, num_base_states + d] = dt
        x_new = A.dot(x)
        self.x = x_new.reshape(-1)
        self.normalize_angle_states()

        self.P = A.dot( self.P.dot( A.T )) + self.Q

        # Update
        self.update(imu_measurement.orientation, imu_measurement.orientation_covariance)

        self.update_lock.release()

        # if self.skip_multiplexer > 250:
        self.publish_estimate(imu_measurement.header.seq, imu_measurement.header.stamp)
        self.skip_multiplexer += 1

    def propagate_strap(self,imu_measurement):
        """
        Propagate state and covariance forward in time by dt using process model
        and IMU measurement.

        Parameters
        ----------
        imu_measurement
            Accelerometer and gyroscope measurements. numpy array of form:
            [a_x, a_y, a_z, w_x, w_y, w_z]
        """
        if self.last_update_time is None:
            self.last_update_time = imu_measurement.header.stamp

            # Instantiate the filter at the first compass meas
            self.x[6:10] = np.array([imu_measurement.orientation.x, \
                                    imu_measurement.orientation.y, \
                                    imu_measurement.orientation.z, \
                                    imu_measurement.orientation.w])
            return
        else:
            delta_t_ros =  imu_measurement.header.stamp - self.last_update_time
            self.last_update_time = imu_measurement.header.stamp
            dt = delta_t_ros.to_sec()
        self.update_lock.acquire()

        # Quaternions
        q0 = np.copy(self.x[6])
        q1 = np.copy(self.x[7])
        q2 = np.copy(self.x[8])
        q3 = np.copy(self.x[9])

        # Biases
        b_ax = self.x[10]
        b_ay = self.x[11]
        b_az = self.x[12]
        b_wx = self.x[13]
        b_wy = self.x[14]
        b_wz = self.x[15]

        # IMU Measurements
        a_x = imu_measurement.linear_acceleration.x
        a_y = imu_measurement.linear_acceleration.y
        a_z = imu_measurement.linear_acceleration.z - G_ACCEL
        w_x = imu_measurement.angular_velocity.x
        w_y = imu_measurement.angular_velocity.y
        w_z = imu_measurement.angular_velocity.z

        # create quaterion rate of change matrix from bias corrected gyro measurements
        quaterion_stm = np.array([[0, -(w_x-b_wx), -(w_y-b_wy), -(w_z-b_wz)],
                                    [w_x-b_wx, 0, w_z-b_wz, -(w_y-b_wy)],
                                    [w_y-b_wy, -(w_z-b_wz), 0, w_x-b_wx],
                                    [w_z-b_wz, w_y-b_wy, -(w_x-b_wx), 0]])

        # propagate attitude quaternion
        q_dot = 0.5*dot(quaterion_stm,np.array([q0,q1,q2,q3]).T)
        # TODO get the angular velocity by substracting
        # the final yaw (euler) from the initial divided by dt
        self.x[6:10] = self.x[6:10] + q_dot*dt

        # Quaternion Normalization
        self.x[6:10] = self.x[6:10]/np.linalg.norm(self.x[6:10])

        q0 = self.x[6]
        q1 = self.x[7]
        q2 = self.x[8]
        q3 = self.x[9]

        body2inertial = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3)],
                                [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                                [2*(q1*q3-q0*q2), 2*(q0*q1+q2*q3), q0**2-q1**2-q2**2+q3**2]])

        # propagate velocity, TODO add RK4
        vel_dot = dot(body2inertial,(np.array([a_x,a_y,a_z])-np.array([b_ax,b_ay,b_az])).T) # - np.array([0,0,G_ACCEL])
        self.x[3:6] = self.x[3:6] + vel_dot*dt

        # propagate position
        pos_dot = self.x[3:6]
        self.x[0:3] = self.x[0:3] + pos_dot*dt

        # derivative of attitude change wrt itself
        F_att_der = 0.5*np.array([[0, b_wx-w_x, b_wy-w_y, b_wz-w_z],
                            [-b_wx+w_x, 0, -b_wz+w_z, b_wy-w_y],
                            [-b_wy+w_y, b_wz-w_z, 0, -b_wx+w_x],
                            [-b_wz+w_z, -b_wy+w_y, b_wx-w_x,0]])

        # derivative of attitude change wrt gyro bias estimate
        F_att_gb_der = 0.5*np.array([[q1,q2,q3],
                            [-q0,q3,-q2],
                            [-q3,-q0,q1],
                            [q2,-q1,-q0]])

        # convenience definitions of bias-corrected IMU acceleration measurements
        al_x = b_ax-a_x; al_y = b_ay-a_y; al_z = b_az-a_z

        # derivative of velocity wrt attitude
        F_vel_att_der = np.array([[2*(q3*al_y-q2*al_z), -2*(q2*al_y+q3*al_z), 2*(2*q2*al_x-q1*al_y-q0*al_z), 2*(2*q3*al_x+q0*al_y-q1*al_z)],
                        [2*(q1*al_z-q3*al_x), 2*(2*q1*al_y-q2*al_x+q0*al_z), -2*(q1*al_x+q3*al_z), 2*(2*q3*al_y-q0*al_x-q2*al_z)],
                        [2*(q2*al_x-q1*al_y), 2*(2*q1*al_z-q3*al_x-q0*al_y), 2*(2*q2*al_z-q3*al_y+q0*al_x), -2*(q1*al_x+q2*al_y)]])

        # derivative of velocity wrt accelerometer bias estimate
        F_vel_ab_der = np.array([[-1+2*(q2**2+q3**2), -2*(q1*q2-q0*q3), -2*(q1*q3+q0*q2)],
                        [-2*(q0*q3+q1*q2), -1+2*(q1**2+q3**2), -2*(q2*q3-q0*q1)],
                        [-2*(q1*q3-q0*q2), -2*(q0*q1+q2*q3), -1+2*(q1**2+q2**2)]])

        # derivative of position wrt velocity
        F_pos_der = np.eye(3)

        # construct linearized dynamics stm
        F = np.zeros([NUM_STATES, NUM_STATES])
        F[0:3,3:6] = F_pos_der
        F[3:6,6:10] = F_vel_att_der
        F[3:6,10:13] = F_vel_ab_der
        F[6:10,6:10] = F_att_der
        F[6:10,13:16] = F_att_gb_der

        # construct process noise matrix
        Q = self.Q * dt # Use covariance matrix from sensor?
    
        # Q[3:6,3:6] = 50*np.array(self.sensors['IMU'].accel_noise)*dt
        # Q[6:10,6:10] = 100*self.sensors['IMU'].gyro_noise[0][0]*np.eye(4)*dt
        # Q[10:13,10:13] = 3*self.sensors['IMU'].accel_bias*dt
        # Q[13:16,13:16] = 7*self.sensors['IMU'].gyro_bias*dt

        stm = np.eye(16) + F*dt
        self.P = dot(stm,dot(self.P,stm.T)) + Q*dt

        # enforce symmetry
        self.P = 0.5*self.P + 0.5*self.P.T

        # renormalize quaternion attitude
        self.x[6:10] = self.x[6:10]/np.linalg.norm(self.x[6:10])

        # Update
        self.update_strapdown(imu_measurement.orientation, imu_measurement.orientation_covariance)

        self.update_lock.release()

        if self.skip_multiplexer > 1000:
            self.publish_estimate_strapdown(imu_measurement.header.seq, imu_measurement.header.stamp, np.zeros(3), np.eye(3)*0.001)

    def update(self, compass_meas, compass_meas_cov):
        if self.last_intersection != None:
            pos = [self.last_intersection.position.x, self.last_intersection.position.y, self.last_intersection.position.z]
            vel = [self.last_intersection.velocity.x, self.last_intersection.velocity.y, self.last_intersection.velocity.z]
            x_CI = np.array([pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]]).reshape(-1,1)
            P_CI = np.array(self.last_intersection.covariance).reshape(6,6)
            self.psci(x_CI, P_CI)

        H = np.zeros((1,NUM_STATES))
        H[0,3] = 1
        _, _, yaw_meas = tf.transformations.euler_from_quaternion([compass_meas.x,\
            compass_meas.y, compass_meas.z, compass_meas.w])
        R = 0.1
        tmp = dot( dot(H, self.P), H.T) + R
        K = dot(self.P, dot( H.T, inv( tmp )) )
        innovation = self.normalize_angle(yaw_meas-self.x[3])
        self.x = self.x + dot(K, innovation).reshape(NUM_STATES)
        self.normalize_angle_states()
        self.P = dot(np.eye(NUM_STATES)-dot(K,H),self.P)

        # Update depth
        if self.last_depth is not None:
            H = np.zeros((1,NUM_STATES))
            H[0,2] = 1
            R = 0.1
            tmp = dot( dot(H, self.P), H.T) + R
            K = dot(self.P, dot( H.T, inv( tmp )) )
            self.x = self.x + dot(K,self.last_depth-self.x[2]).reshape(NUM_STATES)
            self.P = dot(np.eye(NUM_STATES)-dot(K,H),self.P)
            self.last_depth = None

        if self.data_x is not None and self.data_y is not None:
            H = np.zeros((2,NUM_STATES))
            H[0,0] = 1
            H[1,1] = 1
            R = np.eye(2) * 0.1
            meas = np.array([[self.data_x, self.data_y]]).T
            pred = np.array([[self.x[0], self.x[1]]]).T
            tmp = dot( dot(H, self.P), H.T) + R
            K = dot(self.P, dot( H.T, inv( tmp )) )
            self.x = self.x + dot(K,meas-pred).reshape(NUM_STATES)
            self.P = dot(np.eye(NUM_STATES)-dot(K,H),self.P)

            self.data_x = None
            self.data_y = None

        if self.dvl_x is not None and self.dvl_y is not None:
            H = np.zeros((2,NUM_STATES))
            H[0, 4] = 1
            H[1, 5] = 1
            R = np.eye(2) * 0.025
            meas = np.array([[self.dvl_x, self.dvl_y]]).T
            pred = np.array([[self.x[4], self.x[5]]]).T
            tmp = dot( dot(H, self.P), H.T) + R
            K = dot(self.P, dot( H.T, inv( tmp )) )
            self.x = self.x + dot(K,meas-pred).reshape(NUM_STATES)
            self.P = dot(np.eye(NUM_STATES)-dot(K,H),self.P)
            self.dvl_x = None
            self.dvl_y = None

        if self.last_intersection != None:
            self.last_intersection = None
        else:
            self.publish_intersection()
        self.normalize_angle_states()

    def update_strapdown(self, compass_meas, compass_meas_cov):
        
        # _, _, yaw_meas = tf.transformations.euler_from_quaternion([compass_meas.x,\
        #     compass_meas.y, compass_meas.z, compass_meas.w])
        # print(yaw_meas)

        H = np.zeros((4,16))
        H[0:4,6:10] = np.eye(4)

        meas = np.array([[compass_meas.x, compass_meas.y, compass_meas.z, compass_meas.w]]).T
        pred = self.x[6:10].reshape(-1,1)
        R = 0.01
        tmp = dot( dot(H, self.P), H.T) + R
        K = dot(self.P, dot( H.T, inv( tmp )) )
        innovation = meas-pred
        self.x = self.x + dot(K, innovation).reshape(NUM_STATES)
        self.P = dot(np.eye(16)-dot(K,H),self.P)
        # enforce symmetry
        self.P = 0.5*self.P + 0.5*self.P.T
        # renormalize quaternion attitude
        self.x[6:10] = self.x[6:10]/np.linalg.norm(self.x[6:10])

        # Update depth
        if self.last_depth is not None:
            H = np.zeros((1,16))
            H[0,2] = 1
            R = 0.1
            tmp = dot( dot(H, self.P), H.T) + R
            K = dot(self.P, dot( H.T, inv( tmp )) )
            self.x = self.x + dot(K,self.last_depth-self.x[2]).reshape(NUM_STATES)
            self.P = dot(np.eye(16)-dot(K,H),self.P)
            # enforce symmetry
            self.P = 0.5*self.P + 0.5*self.P.T
            # renormalize quaternion attitude
            self.x[6:10] = self.x[6:10]/np.linalg.norm(self.x[6:10])

            self.last_depth = None

        if self.data_x is not None and self.data_y is not None:
            H = np.zeros((2,16))
            H[0,0] = 1
            H[1,1] = 1
            R = np.eye(2) * 0.1
            meas = np.array([[self.data_x, self.data_y]]).T
            pred = np.array([[self.x[0], self.x[1]]]).T
            tmp = dot( dot(H, self.P), H.T) + R
            K = dot(self.P, dot( H.T, inv( tmp )) )
            self.x = self.x + dot(K,meas-pred).reshape(NUM_STATES)
            self.P = dot(np.eye(16)-dot(K,H),self.P)
            # enforce symmetry
            self.P = 0.5*self.P + 0.5*self.P.T
            # renormalize quaternion attitude
            self.x[6:10] = self.x[6:10]/np.linalg.norm(self.x[6:10])

            self.data_x = None
            self.data_y = None

        if self.dvl_x is not None and self.dvl_y is not None:
            H = np.zeros((2,16))
            H[0, 3] = 1
            H[1, 4] = 1
            R = np.eye(2) * 0.025
            meas = np.array([[self.dvl_x, self.dvl_y]]).T
            pred = np.array([[self.x[3], self.x[4]]]).T
            tmp = dot( dot(H, self.P), H.T) + R
            K = dot(self.P, dot( H.T, inv( tmp )) )
            self.x = self.x + dot(K,meas-pred).reshape(NUM_STATES)
            self.P = dot(np.eye(16)-dot(K,H),self.P)
            # enforce symmetry
            self.P = 0.5*self.P + 0.5*self.P.T
            # renormalize quaternion attitude
            self.x[6:10] = self.x[6:10]/np.linalg.norm(self.x[6:10])
            self.dvl_x = None
            self.dvl_y = None

        if self.last_intersection != None:
            self.cuprint("Received CI result, correcting")
            pos = [self.last_intersection.position.x, self.last_intersection.position.y, self.last_intersection.position.z]
            vel = [self.last_intersection.velocity.x, self.last_intersection.velocity.y, self.last_intersection.velocity.z]
            x_CI = np.array([pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]]).reshape(-1,1)
            P_CI = np.array(self.last_intersection.covariance).reshape(6,6)
            x_prior = self.x[:6].reshape(-1,1)
            P_prior = self.P[:6, :6]
            self.psci_strapdown(x_prior, P_prior, x_CI, P_CI) # LOOK BACK IN HISTORY
            self.last_intersection = None
        else:
            self.publish_intersection()

        # Artificially give some zero velocity measurements for the first 30s
        # if self.skip_multiplexer < 2000:
        #     if self.skip_multiplexer > 1000:
        #         rospy.loginfo_once("################################Ready to dive###################################")
        #     H = np.zeros((2,16))
        #     H[0, 3] = 1
        #     H[1, 4] = 1
        #     R = np.eye(2) * 0.01
        #     meas = np.array([[0.0, 0.0]]).T
        #     pred = np.array([[self.x[3], self.x[4]]]).T
        #     tmp = dot( dot(H, self.P), H.T) + R
        #     K = dot(self.P, dot( H.T, inv( tmp )) )
        #     self.x = self.x + dot(K,meas-pred).reshape(NUM_STATES)
        #     self.P = dot(np.eye(16)-dot(K,H),self.P)
        #     # enforce symmetry
        #     self.P = 0.5*self.P + 0.5*self.P.T
        #     # renormalize quaternion attitude
        #     self.x[6:10] = self.x[6:10]/np.linalg.norm(self.x[6:10])
        # else:
        #     rospy.loginfo_once("Fake DVL meas done")

    def psci(self, x_CI, P_CI):
        # Transformation matrix from strapdown state ordering to common information state ordering
        trans_strap2ci = np.zeros((8,8))
        trans_strap2ci[:3,:3] = np.eye(3)
        trans_strap2ci[3:6, 4:7] = np.eye(3)
        trans_strap2ci[6,3] = 1
        trans_strap2ci[7,7] = 1

        x = dot(trans_strap2ci, self.x.reshape(-1,1))
        P = dot(trans_strap2ci, self.P).dot(trans_strap2ci.T)

        simple = False
        if simple:
            x[:6] = x_CI
            P[:6,:6] = P_CI
            self.x = dot(trans_strap2ci.T, x).reshape(-1)
            self.P = dot(trans_strap2ci.T, P).dot(trans_strap2ci)
        else:
            x_cij = x[:6]
            P_cij = P[:6,:6]

            """

            NOTE IS DIFFERENT FROM THE THESIS, SWITHC!!!!!!!

            """

            # D_inv = inv( P_cij ) - inv(P_CI)
            # D_inv_d = dot( inv( P_cij ), x_cij) - dot( inv(P_CI), x_CI)
            D_inv = inv( P_CI ) - inv(P_cij)
            D_inv_d = dot( inv( P_CI ), x_CI) - dot( inv(P_cij), x_cij)

            info_matrix = np.zeros((8,8))
            info_matrix[:6,:6] = D_inv
            info_vector = np.zeros((8,1))
            info_vector[:6] = D_inv_d
            
            posterior_cov = inv( inv( P ) + info_matrix )
            tmp = dot(inv( P ), x) + info_vector
            posterior_state = dot( posterior_cov, tmp )

            # posterior_state[:6] = x_CI
            # posterior_cov[:6,:6] = P_CI

            self.x = dot(trans_strap2ci.T, posterior_state).reshape(-1)
            self.P = dot(trans_strap2ci.T, posterior_cov).dot(trans_strap2ci)

        

    def psci_strapdown(self, x_prior, P_prior, x_CI, P_CI):
        x = self.x.reshape(-1,1)
        P = self.P

        D_inv = inv(P_prior) - inv(P_CI)
        D_inv_d = dot( inv(P_prior), x_prior) - dot( inv(P_CI), x_CI)

        info_vector = np.zeros( x.shape )
        info_vector[:6] = D_inv_d

        info_matrix = np.zeros( P.shape )
        info_matrix[:6,:6] = D_inv

        posterior_cov = inv( inv( P ) + info_matrix )
        tmp = dot(inv( P ), x) + info_vector
        posterior_state = dot( posterior_cov, tmp )

        self.x = posterior_state.reshape(-1)
        self.P = posterior_cov


def get_initial_estimate():
    starting_position = rospy.get_param("~starting_position")
    starting_yaw = rospy.get_param("~starting_yaw")
    initial_uncertainty = rospy.get_param("~initial_uncertainty/ownship")
    state = np.zeros(NUM_STATES)
    state[0:3] = starting_position[0:3]
    state[3] = starting_yaw
    uncertainty = np.zeros((NUM_STATES, NUM_STATES))
    uncertainty[0,0] = initial_uncertainty["x"]
    uncertainty[1,1] = initial_uncertainty["y"]
    uncertainty[2,2] = initial_uncertainty["z"]
    uncertainty[3,3] = initial_uncertainty["yaw"]
    uncertainty[4,4] = initial_uncertainty["x_vel"]
    uncertainty[5,5] = initial_uncertainty["y_vel"]
    uncertainty[6,6] = initial_uncertainty["z_vel"]
    uncertainty[7,7] = initial_uncertainty["yaw_vel"]
    return state, uncertainty


def get_initial_estimate_strapdown():
    starting_position = rospy.get_param("~starting_position")
    starting_yaw = rospy.get_param("~starting_yaw")
    initial_uncertainty = rospy.get_param("~initial_uncertainty/ownship")
    quaternion = tf.transformations.quaternion_from_euler(0, 0, starting_yaw)
    state = np.zeros(NUM_STATES)
    state[0:3] = starting_position[0:3]
    state[5:9] = np.array(quaternion)
    uncertainty = np.zeros((NUM_STATES, NUM_STATES))
    uncertainty[0,0] = initial_uncertainty["x"]
    uncertainty[1,1] = initial_uncertainty["y"]
    uncertainty[2,2] = initial_uncertainty["z"]
    uncertainty[3,3] = initial_uncertainty["x_vel"]
    uncertainty[4,4] = initial_uncertainty["y_vel"]
    uncertainty[5,5] = initial_uncertainty["z_vel"]
    uncertainty[6:10, 6:10] = np.eye(4) * initial_uncertainty["quat"]
    uncertainty[10:16, 10:16] = np.eye(6) * initial_uncertainty["bias"]
    return state, uncertainty

def get_process_noise():
    Q = np.zeros((NUM_STATES, NUM_STATES))
    ownship_Q = rospy.get_param("~process_noise/ownship")
    Q[0,0] = ownship_Q["x"]
    Q[1,1] = ownship_Q["y"]
    Q[2,2] = ownship_Q["z"]
    Q[3,3] = ownship_Q["yaw"]
    Q[4,4] = ownship_Q["x_vel"]
    Q[5,5] = ownship_Q["y_vel"]
    Q[6,6] = ownship_Q["z_vel"]
    Q[7,7] = ownship_Q["yaw_vel"]
    return Q

def get_procdess_noise_strapdown():
    Q = np.zeros((NUM_STATES, NUM_STATES))
    ownship_Q = rospy.get_param("~process_noise/ownship")
    Q[0,0] = ownship_Q["x"]
    Q[1,1] = ownship_Q["y"]
    Q[2,2] = ownship_Q["z"]
    Q[3,3] = ownship_Q["x_vel"]
    Q[4,4] = ownship_Q["y_vel"]
    Q[5,5] = ownship_Q["z_vel"]
    Q[6:10,6:10] = 100*ownship_Q["quat"]*np.eye(4)
    Q[10:13,10:13] = 3*ownship_Q["accel_bias"] * np.eye(3)
    Q[13:16,13:16] = 7*ownship_Q["gyro_bias"] * np.eye(3)
    return Q

def get_default_meas_variance():
    meas_vars = {}
    meas_info = rospy.get_param("~measurements")
    for meas in meas_info.keys():
        sd = meas_info[meas]["default_sd"]
        meas_vars[meas] = sd ** 2
    return meas_vars

if __name__ == "__main__":
    rospy.init_node("strapdown")

    # strapdown = rospy.get_param("~strapdown")
    strapdown = False
    if strapdown:
        x0, P0 = get_initial_estimate_strapdown()
        Q = get_process_noise_strapdown()
    else:
        NUM_STATES = 8
        x0, P0 = get_initial_estimate()
        Q = get_process_noise()
    
    # default_meas_variance = get_default_meas_variance()
    default_meas_variance = {}
    # use_control_input = rospy.get_param("~use_control_input")

    s = StrapdownINS(x0, P0, Q, default_meas_variance, strapdown)
    rospy.spin()