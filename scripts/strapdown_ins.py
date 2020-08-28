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
from scipy.linalg import block_diag, sqrtm
import tf
import threading

import sys

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
    def __init__(self, x0, P0, Q, default_meas_variance):

        self.last_update_time = None
        self.update_lock = threading.Lock()
        # initialize estimate and covariance
        self.x = x0.reshape(-1)
        self.P = P0
        self.Q = Q
        self.pub = rospy.Publisher("strapdown/estimate", Odometry, queue_size=10)
        self.last_depth = None
        self.data_x, self.data_y = None, None
        self.skip_multiplexer = 0
        rospy.sleep(rospy.get_param("~wait_on_startup"))
        rospy.Subscriber("mavros/global_position/rel_alt", Float64, self.depth_callback)
        rospy.Subscriber("pose_gt", Odometry, self.gps_callback)
        rospy.Subscriber("imu", Imu, self.propagate)
        rospy.loginfo("Loaded")

    def gps_callback(self, msg):
        self.skip_multiplexer += 1
        if self.skip_multiplexer % 300 == 0:
            self.data_x = msg.pose.pose.position.x
            self.data_y = msg.pose.pose.position.y

    def depth_callback(self, msg):
        self.last_depth = msg.data

    def publish_estimate(self, update_seq, timestamp, angular_velocity, angular_velocity_cov):
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

    def propagate(self,imu_measurement):
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
        a_x = -imu_measurement.linear_acceleration.x
        a_y = -imu_measurement.linear_acceleration.y
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
        q_dot = 0.5*np.dot(quaterion_stm,np.array([q0,q1,q2,q3]).T)
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
        vel_dot = np.dot(body2inertial,(np.array([a_x,a_y,a_z])-np.array([b_ax,b_ay,b_az])).T) # - np.array([0,0,G_ACCEL])
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
        self.P = np.dot(stm,np.dot(self.P,stm.T)) + Q*dt

        # enforce symmetry
        self.P = 0.5*self.P + 0.5*self.P.T

        # renormalize quaternion attitude
        self.x[6:10] = self.x[6:10]/np.linalg.norm(self.x[6:10])

        # Update
        self.update(imu_measurement.orientation, imu_measurement.orientation_covariance)

        self.update_lock.release()

        if self.skip_multiplexer > 1000:
            self.publish_estimate(imu_measurement.header.seq, imu_measurement.header.stamp, np.zeros(3), np.eye(3)*0.001)

    def update(self, compass_meas, compass_meas_cov):
        
        # _, _, yaw_meas = tf.transformations.euler_from_quaternion([compass_meas.x,\
        #     compass_meas.y, compass_meas.z, compass_meas.w])
        # print(yaw_meas)

        H = np.zeros((4,16))
        H[0:4,6:10] = np.eye(4)

        meas = np.array([[compass_meas.x, compass_meas.y, compass_meas.z, compass_meas.w]]).T
        pred = self.x[6:10].reshape(-1,1)
        R = 0.01
        tmp = np.dot( np.dot(H, self.P), H.T) + R
        K = np.dot(self.P, np.dot( H.T, np.linalg.inv( tmp )) )
        innovation = meas-pred
        self.x = self.x + np.dot(K, innovation).reshape(NUM_STATES)
        self.P = np.dot(np.eye(16)-np.dot(K,H),self.P)
        # enforce symmetry
        self.P = 0.5*self.P + 0.5*self.P.T
        # renormalize quaternion attitude
        self.x[6:10] = self.x[6:10]/np.linalg.norm(self.x[6:10])

        # Update depth
        if self.last_depth is not None:
            H = np.zeros((1,16))
            H[0,2] = 1
            R = 0.3
            tmp = np.dot( np.dot(H, self.P), H.T) + R
            K = np.dot(self.P, np.dot( H.T, np.linalg.inv( tmp )) )
            self.x = self.x + np.dot(K,self.last_depth-self.x[2]).reshape(NUM_STATES)
            self.P = np.dot(np.eye(16)-np.dot(K,H),self.P)
            # enforce symmetry
            self.P = 0.5*self.P + 0.5*self.P.T
            # renormalize quaternion attitude
            self.x[6:10] = self.x[6:10]/np.linalg.norm(self.x[6:10])

            self.last_depth = None

        if self.data_x is not None and self.data_y is not None:
            H = np.zeros((2,16))
            H[0,0] = 1
            H[1,1] = 1
            R = 0.01
            meas = np.array([[self.data_x, self.data_y]]).T
            pred = np.array([[self.x[0], self.x[1]]]).T
            tmp = np.dot( np.dot(H, self.P), H.T) + R
            K = np.dot(self.P, np.dot( H.T, np.linalg.inv( tmp )) )
            self.x = self.x + np.dot(K,meas-pred).reshape(NUM_STATES)
            self.P = np.dot(np.eye(16)-np.dot(K,H),self.P)
            # enforce symmetry
            self.P = 0.5*self.P + 0.5*self.P.T
            # renormalize quaternion attitude
            self.x[6:10] = self.x[6:10]/np.linalg.norm(self.x[6:10])

            self.data_x = None
            self.data_y = None

        # Artificially give some zero velocity measurements for the first 30s
        if self.skip_multiplexer < 1000:
            H = np.zeros((3,16))
            H[0, 3] = 1
            H[1, 4] = 1
            H[2, 5] = 1
            R = 0.01
            meas = np.array([[0.0, 0.0, 0.0]]).T
            pred = np.array([[self.x[3], self.x[4], self.x[5]]]).T
            tmp = np.dot( np.dot(H, self.P), H.T) + R
            K = np.dot(self.P, np.dot( H.T, np.linalg.inv( tmp )) )
            self.x = self.x + np.dot(K,meas-pred).reshape(NUM_STATES)
            self.P = np.dot(np.eye(16)-np.dot(K,H),self.P)
            # enforce symmetry
            self.P = 0.5*self.P + 0.5*self.P.T
            # renormalize quaternion attitude
            self.x[6:10] = self.x[6:10]/np.linalg.norm(self.x[6:10])
        else:
            rospy.loginfo_once("Fake DVL meas done")

    #     if measurement_type == 'GPS':
    #         H = np.zeros((3,16))
    #         H[0:3,0:3] = np.eye(3)
    #         R = self.sensors['GPS'].noise
    #         h = self.x[0:3]
    #         # self.gps_residuals = np.concatenate((self.gps_residuals,measurement-h))
    #     elif measurement_type == 'Depth':
    #         H = np.zeros((1,16))
    #         H[0,2] = 1
    #         R = self.sensors['Depth'].noise
    #         h = self.x[2]
    #         # self.depth_residuals = np.concatenate((self.depth_residuals,measurement-h))
    #     elif measurement_type == 'GPS_x':
    #         H = np.zeros((1,16))
    #         H[0,0] = 1
    #         R = self.sensors['GPS'].noise[0,0]
    #         h = self.x[0]
    #     elif measurement_type == 'GPS_y':
    #         H = np.zeros((1,16))
    #         H[0,1] = 1
    #         R = self.sensors['GPS'].noise[1,1]
    #         h = self.x[1]
    #     elif measurement_type == 'GPS_z':
    #         H = np.zeros((1,16))
    #         H[0,2] = 1
    #         R = self.sensors['GPS'].noise[2,2]
    #         h = self.x[2]
        # elif measurement_type == 'Compass':
            # H = np.zeros((1,16))
            # H[0:4,6:10] = np.eye(4)
            

    #     elif measurement_type == 'DVL':
    #         H = np.zeros((3,16))
    #         H[0:3,3:6] = np.eye(3)
    #         R = self.sensors['DVL'].noise
    #         h = self.x[3:6]
    #     elif measurement_type == 'Magnetometer':
    #         H = np.zeros((4,16))
    #         H[0:4,6:10] = np.eye(4)
    #         R = np.sqrt(np.diag(self.sensors['Magnetometer'].noise))[0]**2*np.eye(4)
    #         h = self.x[6:10]
    #         measurement = self.euler2quat(measurement)

    #     # compute the Kalman gain for the measurement
    
        
    #     try:
    #         assert(K.shape == (self.x.shape[0],H.shape[0]))
    #     except AssertionError:
    #         print('K is the wrong shape!: Is {}, should be {}'.format(K.shape,(self.x.shape[0],H.shape[0])))
    #         raise AssertionError

    

def get_initial_estimate():
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

    x0, P0 = get_initial_estimate()
    Q = get_process_noise()
    # default_meas_variance = get_default_meas_variance()
    default_meas_variance = {}
    # use_control_input = rospy.get_param("~use_control_input")

    s = StrapdownINS(x0, P0, Q, default_meas_variance)
    rospy.spin()
"""
That propagate functions looks fine


I'll have 8h tomorrow to:
1) Get the navigation filter working: add compass, GPS, DVL
2) Make it work with ETDDF & the PSCI
3) Record some data navigating single vehicle autonomously
4) Run experiment tracking team members (may need to write a measurement buffer transporter)
5) Run experiment tracking red agent
6) Repeat experiment, collect some figures
7) Work with Robert to debug sharing of measurements
8) MONEY: Run experiment with tracking 8 agents using delta tiering vs CI (doesn't even need red agent)
---> Nisar would be very interested in this

Trim the fat
Add RK4 integration
I'm probably going to need to tune the gyro + accelerometer noise
Test deadreckoning with perfect acceleration measurements, add magnetometer measurements (no noise)
Add in some biases w/ periodic truth measurements
move this file to etddf repo
Add covariance intersection support (take in an odom message)
Run the test

Do covariance intersection on the strapdown side

FIRST:
Take in no noise GYRO + Accel data, see how well it does deadreckoning (if it does well halleyluyah!)
Add in compass data, see if we still get good results
TEST
SECOND: (<20min)
Add in noise see how well it does
TEST

TEST out sonar on the bluerov

have etddf take in pose_gt (components as GPS)
Now do CI with PS-CI
TEST, we should the nav filter perform excellently

THIRD:
Do PS-CI on the etddf side
Have etddf publish at 1-2Hz
Test it all out
Tie nav filter to uuv control

FOURTH:
Put it on hardware and check our velocity estimates

Slow down ETDDF to publish at 1-2Hz
"""