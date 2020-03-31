from __future__ import division
import numpy as np
from measurements import *
from scipy.stats import norm as normal
from copy import deepcopy
from pdb import set_trace
from scipy import integrate

DEBUG=False

class ETFilter(object):

    def __init__(self, my_id, num_ownship_states, world_dim, x0, P0, linear_dynamics):
        self.my_id = my_id
        self.num_ownship_states = num_ownship_states
        self.world_dim = world_dim
        self.x_hat = deepcopy(x0)
        self.P = deepcopy(P0)
        self.linear_dynamics = linear_dynamics
        self.num_states = self.x_hat.size
        if (self.num_states % self.num_ownship_states) != 0:
            raise Exception("Dimensionality of state vector does not align with the number of ownship states.")
        elif num_ownship_states < world_dim:
            raise Exception("Number of ownship states does not make sense for world dimension")
        self.num_assets = int( self.num_states / self.num_ownship_states )
        self.is_main_fitler = False
        self.meas_queue = []

    def check_implicit(self, meas):
        if not isinstance(meas, Measurement):
            raise Exception("meas must of type Measurement")
        if self._is_angle_meas(meas):
            meas.data = self._normalize_angle(meas.data)

        C = self._get_measurement_jacobian(meas)
        innovation = self._get_innovation(meas, C)
        return np.abs(innovation) <= meas.et_delta

    def add_meas(self, meas):
        if DEBUG:
            print(str(self.my_id) + " receiving meas: " + meas.__class__.__name__ + " | data: " + str(meas.data))

        if not isinstance(meas, Measurement):
            raise Exception("meas must of type Measurement")
        if self._is_angle_meas(meas):
            meas.data = self._normalize_angle(meas.data)

        self.meas_queue.append(meas)

        # Check if this is our first meas of asset
        # If the meas is a range/bearing, instantiate the asset at measurement mean
        if isinstance(meas, Azimuth_Explicit) or isinstance(meas, Range_Explicit):
            if self.P[meas.measured_asset*self.num_ownship_states, meas.measured_asset*self.num_ownship_states] > (100/2)**2:
                self._instantiate_asset_range_bearing(meas.measured_asset)
    
    def predict(self, u, Q):
        if u.shape[1] > u.shape[0]:
            raise Exception("u must be column vector")
        if Q.shape != self.P.shape:
            raise Exception("Q must have (state x state) dimensions")

        if self.linear_dynamics:
            A = self._linear_propagation(u)
            self.P = A.dot( self.P.dot( A.T )) + Q
        else: # nonlinear dynamics
            G = self._nonlinear_propagation(u)
            self.P = G.dot( self.P.dot( G.T )) + Q
        self.x_hat = self._normalize_all_angles(self.x_hat)

    def correct(self):
        if not self.meas_queue:
            # print("meas_queue is empty!")
            return

        x_hat_start = self.x_hat
        P_start = self.P
        for meas in self.meas_queue:
            if DEBUG:
                print("Fusing " + meas.__class__.__name__ + " w/ data: " + str(meas.data))
                print("State of Filter:")
                print("x_hat")
                print(self.x_hat)
                print("P")
                print(self.P)

            K, C = None, None
            R = meas.R
            if isinstance(meas, Explicit):
                C = self._get_measurement_jacobian(meas)
                K = self._get_kalman_gain(C, R)
                innovation = self._get_innovation(meas, C).reshape(1,1)                
                self.x_hat += np.dot( K, innovation)
                self.P = ( np.eye(self.num_states) - np.dot(K, C) ).dot(self.P)
            else: # Implicit Update
                C = self._get_measurement_jacobian(meas)
                mu, Qe, alpha = self._get_implicit_predata(C, R, x_hat_start, P_start, meas)
                z_bar, curly_theta = self._get_implicit_data(meas.et_delta, mu, Qe, alpha)
                K = self._get_kalman_gain(C, R)
                if self._is_angle_meas(meas, check_implicit=True):
                    z_bar = self._normalize_angle(z_bar)
                self.x_hat += np.dot(K, z_bar)
                self.P = self.P - curly_theta * K.dot(C.dot(self.P))
            self.x_hat = self._normalize_all_angles( self.x_hat )
            # if isinstance(meas, Azimuth_Explicit) and self.my_id == 1 and meas.measured_asset == 2:
            #     print("Just fused: " + meas.__class__.__name__)
            #     print("new state: \n" + str(self.x_hat))
            #     red_team_u = self.P[2*self.num_ownship_states:2*self.num_ownship_states+2, 2*self.num_ownship_states:2*self.num_ownship_states+2]
            #     print("Red Team Uncertainty: \n" + str(2*np.sqrt(red_team_u)))
            #     raise Exception("ya done!")
        # Clear measurement queue
        self.meas_queue = []

    # If ETFilter_Main, method is overridden
    def _get_implicit_predata(self, C, R, x_hat_start, P_start, meas):
        mu = alpha = 0 # mu, alpha cancel out in the common information filter, not the case in main filter
        Qe = np.abs( C.dot( P_start.dot( C.T )) + R )
        return mu, Qe, alpha

    def _get_innovation(self, meas, C):
        expected_meas = None
        if meas.is_linear_meas:
            expected_meas = C.dot(self.x_hat)
        else:
            expected_meas = self._get_nonlinear_expected_meas(meas, self.x_hat)
        if DEBUG:
            print("expected meas " + meas.__class__.__name__ + " : " + str(expected_meas))
            print("---> actual meas: " + str(meas.data))
        if self._is_angle_meas(meas):
            return self._normalize_angle( meas.data - expected_meas)
        else:
            return meas.data - expected_meas

    def _get_kalman_gain(self, C, R):
        tmp = np.dot( np.dot(C, self.P), C.T ) + R
        tmp_inv = np.linalg.inv( tmp ) if tmp.size > 1 else tmp**(-1)
        return self.P.dot(C.T.dot( tmp_inv ))

    def _get_implicit_data(self, delta, mu, Qe, alpha):
        Q_func = lambda x : 1 - normal.cdf(x)
        
        arg1 = ( -delta + alpha - mu ) / np.sqrt( Qe )
        arg2 = ( delta + alpha - mu ) / np.sqrt( Qe )

        tmp = ( normal.pdf( arg1 ) - normal.pdf( arg2 ) ) / ( Q_func(arg1) - Q_func(arg2 ) )
        z_bar = tmp.dot( np.sqrt( Qe ) )

        tmp2 = ( arg1.dot( normal.pdf( arg1)) - arg2.dot( normal.pdf( arg2 )) ) / ( Q_func(arg1) - Q_func(arg2))
        curly_theta = np.linalg.matrix_power(tmp, 2) - tmp2

        return z_bar, curly_theta

    def _get_measurement_jacobian(self, meas):
        C = np.zeros((1, self.num_states))
        src_id = meas.src_id
        if isinstance(meas, GPSx_Explicit) or isinstance(meas, GPSx_Implicit):
            C[0, src_id*self.num_ownship_states] = 1
        elif isinstance(meas, GPSy_Explicit) or isinstance(meas, GPSy_Implicit):
            C[0, src_id*self.num_ownship_states + 1] = 1
        elif isinstance(meas, GPSyaw_Explicit) or isinstance(meas, GPSyaw_Implicit):
            if self.world_dim == 2:
                C[0, src_id*self.num_ownship_states + 2] = 1
            else: # world dim 3
                C[0, src_id*self.num_ownship_states + 5] = 1
        elif isinstance(meas, GPSx_Neighbor_Explicit) or isinstance(meas, GPSx_Neighbor_Implicit):
            C[0, meas.neighbor_id*self.num_ownship_states] = 1
        elif isinstance(meas, GPSy_Neighbor_Explicit) or isinstance(meas, GPSy_Neighbor_Implicit):
            C[0, meas.neighbor_id*self.num_ownship_states+1] = 1
        elif isinstance(meas, GPSyaw_Neighbor_Explicit) or isinstance(meas, GPSyaw_Neighbor_Implicit):
            C[0, meas.neighbor_id*self.num_ownship_states+2] = 1
        elif isinstance(meas, LinRelx_Explicit) or isinstance(meas, LinRelx_Implicit):
            C[0, src_id*self.num_ownship_states] = -1
            C[0, meas.measured_asset*self.num_ownship_states] = 1
        elif isinstance(meas, LinRely_Explicit) or isinstance(meas, LinRely_Implicit):
            C[0, src_id*self.num_ownship_states + 1] = -1
            C[0, meas.measured_asset*self.num_ownship_states + 1] = 1
        elif isinstance(meas, Azimuth_Explicit) or isinstance(meas, Azimuth_Implicit):
            meas_id = meas.measured_asset
            if self.world_dim == 2:
                src_x = self.x_hat[src_id*self.num_ownship_states,0]
                src_y = self.x_hat[src_id*self.num_ownship_states+1,0]
                other_x = self.x_hat[meas_id*self.num_ownship_states,0]
                other_y = self.x_hat[meas_id*self.num_ownship_states+1,0]
                diff_x = other_x - src_x
                diff_y = other_y - src_y

                # Protect division by zero
                diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
                diff_y = diff_y if abs(diff_y) > 0.01 else 0.01

                C[0, src_id*self.num_ownship_states + 2] = -1
                # Bearing jacobians...
                C[0, src_id*self.num_ownship_states] = diff_y / ( diff_x**2 + diff_y**2 )
                C[0, meas_id*self.num_ownship_states] = -diff_y / ( diff_x**2 + diff_y**2 )
                C[0, src_id*self.num_ownship_states+1] = -diff_x / ( diff_x**2 + diff_y**2 )
                C[0, meas_id*self.num_ownship_states+1] = diff_x / ( diff_x**2 + diff_y**2 )
        elif isinstance(meas, AzimuthGlobal_Explicit) or isinstance(meas, AzimuthGlobal_Implicit):
            if self.world_dim == 2:
                src_x = self.x_hat[src_id*self.num_ownship_states,0]
                src_y = self.x_hat[src_id*self.num_ownship_states+1,0]
                other_x = meas.global_pos[0]
                other_y = meas.global_pos[1]
                diff_x = other_x - src_x
                diff_y = other_y - src_y

                # Protect division by zero
                diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
                diff_y = diff_y if abs(diff_y) > 0.01 else 0.01

                C[0, src_id*self.num_ownship_states + 2] = -1
                # Bearing jacobians...
                C[0, src_id*self.num_ownship_states] = diff_y / ( diff_x**2 + diff_y**2 )
                C[0, src_id*self.num_ownship_states+1] = -diff_x / ( diff_x**2 + diff_y**2 )
        elif isinstance(meas, Range_Explicit) or isinstance(meas, Range_Implicit):
            meas_id = meas.measured_asset
            if self.world_dim == 2:
                src_x = self.x_hat[src_id*self.num_ownship_states,0]
                src_y = self.x_hat[src_id*self.num_ownship_states+1,0]
                other_x = self.x_hat[meas.measured_asset*self.num_ownship_states,0]
                other_y = self.x_hat[meas.measured_asset*self.num_ownship_states+1,0]
                diff_x = other_x - src_x
                diff_y = other_y - src_y
                r = np.sqrt( diff_x**2 + diff_y**2 )
                r = r if r > 0.01 else 0.01 # Division by zero protection
                C[0, src_id*self.num_ownship_states] = -diff_x / r
                C[0, meas_id*self.num_ownship_states] = diff_x / r
                C[0, src_id*self.num_ownship_states+1] = -diff_y / r
                C[0, meas_id*self.num_ownship_states+1] = diff_y / r

        elif isinstance(meas, RangeGlobal_Explicit) or isinstance(meas, RangeGlobal_Implicit):
            if self.world_dim == 2:
                src_x = self.x_hat[src_id*self.num_ownship_states,0]
                src_y = self.x_hat[src_id*self.num_ownship_states+1,0]
                other_x = meas.global_pos[0]
                other_y = meas.global_pos[1]
                diff_x = other_x - src_x
                diff_y = other_y - src_y
                r = np.sqrt( diff_x**2 + diff_y**2 )
                r = r if r > 0.01 else 0.01 # Division by zero protection
                C[0, src_id*self.num_ownship_states] = -diff_x / r
                C[0, src_id*self.num_ownship_states+1] = -diff_y / r

        # elif isinstance(meas, LinRely_Explicit) or isinstance(meas, LinRely_Implicit):
        #     C = np.zeros((1, self.num_states))
        #     C[0, src_id*self.world_dim + 1] = -1
        #     C[0, meas.measured_asset*self.world_dim + 1] = 1
        else:
            raise NotImplementedError("Measurment Jacobian not implemented for: " + meas.__class__.__name__)
        return C

    def _get_nonlinear_expected_meas(self, meas, x_hat):
        if isinstance(meas, Azimuth_Explicit) or isinstance(meas, Azimuth_Implicit):
            if self.world_dim == 2:
                src_bearing = x_hat[meas.src_id*self.num_ownship_states + 2,0]
                src_x = x_hat[meas.src_id*self.num_ownship_states,0]
                src_y = x_hat[meas.src_id*self.num_ownship_states+1,0]
                other_x = x_hat[meas.measured_asset*self.num_ownship_states,0]
                other_y = x_hat[meas.measured_asset*self.num_ownship_states+1,0]
                diff_x = other_x - src_x
                diff_y = other_y - src_y

                # Protect division by zero
                diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
                diff_y = diff_y if abs(diff_y) > 0.01 else 0.01
                expected_bearing = np.arctan2(diff_y, diff_x) - src_bearing
                return self._normalize_angle( expected_bearing )
        elif isinstance(meas, AzimuthGlobal_Explicit) or isinstance(meas, AzimuthGlobal_Implicit):
            if self.world_dim == 2:
                src_bearing = x_hat[meas.src_id*self.num_ownship_states + 2,0]
                src_x = x_hat[meas.src_id*self.num_ownship_states,0]
                src_y = x_hat[meas.src_id*self.num_ownship_states+1,0]
                other_x = meas.global_pos[0]
                other_y = meas.global_pos[1]
                diff_x = other_x - src_x
                diff_y = other_y - src_y

                # Protect division by zero
                diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
                diff_y = diff_y if abs(diff_y) > 0.01 else 0.01
                expected_bearing = np.arctan2(diff_y, diff_x) - src_bearing
                return self._normalize_angle( expected_bearing )
        elif isinstance(meas, Range_Explicit) or isinstance(meas, Range_Implicit):
            if self.world_dim == 2:
                src_x = x_hat[meas.src_id*self.num_ownship_states,0]
                src_y = x_hat[meas.src_id*self.num_ownship_states+1,0]
                other_x = x_hat[meas.measured_asset*self.num_ownship_states,0]
                other_y = x_hat[meas.measured_asset*self.num_ownship_states+1,0]
                diff_x = other_x - src_x
                diff_y = other_y - src_y
                return np.sqrt( diff_x**2 + diff_y**2 )
        elif isinstance(meas, RangeGlobal_Explicit) or isinstance(meas, RangeGlobal_Implicit):
            if self.world_dim == 2:
                src_x = x_hat[meas.src_id*self.num_ownship_states,0]
                src_y = x_hat[meas.src_id*self.num_ownship_states+1,0]
                other_x = meas.global_pos[0]
                other_y = meas.global_pos[1]
                diff_x = other_x - src_x
                diff_y = other_y - src_y
                return np.sqrt( diff_x**2 + diff_y**2 )
        else:
            raise NotImplementedError("Nonlinear Measurement Innovation not implemented for: " + meas.__class__.__name__)

    def _instantiate_asset_range_bearing(self, asset_id):
        if self.world_dim == 2:
            range_meas = [x for x in self.meas_queue if (isinstance(x, Range_Explicit) and x.measured_asset == asset_id)]
            az_meas = [x for x in self.meas_queue if (isinstance(x, Azimuth_Explicit) and x.measured_asset == asset_id)]
            if not range_meas or not az_meas: # still waiting on other meas
                return
            else: # instantiate gaussian of asset at measurement mean
                r = range_meas[0].data
                az = az_meas[0].data
                # print("range: " + str(r))
                # print("az: " + str(az))
                src_id = range_meas[0].src_id
                src_yaw = self.x_hat[ src_id * self.num_ownship_states + 2,0]
                cov_ori = az + src_yaw
                mean_x = r * np.cos(cov_ori) + self.x_hat[src_id*self.num_ownship_states,0]
                mean_y = r * np.sin(cov_ori) + self.x_hat[src_id*self.num_ownship_states+1,0]
                prev_state = deepcopy(self.x_hat)
                # Find covariance from eigendata
                r_var = range_meas[0].R
                az_var = range_meas[0].R
                eig_az = r * np.tan(az_var)
                e_vec = np.array([[np.cos(cov_ori), np.sin(cov_ori)]]).T
                e_vec2 = np.array([[np.cos(cov_ori + np.pi/2), np.sin(cov_ori + np.pi/2)]]).T
                S = np.concatenate((e_vec, e_vec2), axis=1)
                D = np.zeros((2,2)); D[0,0] = r_var; D[1,1] = eig_az
                A = np.dot( np.dot(S,D), np.linalg.inv(S) )
                self.x_hat[asset_id*self.num_ownship_states,0] = mean_x
                self.x_hat[asset_id*self.num_ownship_states+1,0] = mean_y

                # Zero out cov columns
                for col in range(self.num_ownship_states):
                    self.P[:, asset_id*self.num_ownship_states+col] = np.zeros(self.num_states)
                for row in range(self.num_ownship_states):
                    self.P[asset_id*self.num_ownship_states+row,:] = np.zeros(self.num_states)
                self.P[asset_id*self.num_ownship_states:asset_id*self.num_ownship_states+2, asset_id*self.num_ownship_states:asset_id*self.num_ownship_states+2] = A
                self.P[asset_id*self.num_ownship_states+2,asset_id*self.num_ownship_states+2] = 9
                self.meas_queue.remove(range_meas[0])
                self.meas_queue.remove(az_meas[0])

                # print("instantiating asset at " + str(self.x_hat[asset_id*self.num_ownship_states:asset_id*self.num_ownship_states+2,0]))
                # print("rel x: "+ str(r * np.cos(cov_ori)))
                # print("rel y: " + str(r * np.sin(cov_ori)))
                # print("prev state: \n" + str(prev_state))
                # raise Exception("ya done")
    
    # Normalize Angle -pi to pi
    def _normalize_angle(self, angle):
        return np.mod( angle + np.pi, 2*np.pi) - np.pi
    # Normalize all angles in our state
    def _normalize_all_angles(self, state_vector):
        if self.world_dim == 2 and self.num_ownship_states in [3,6]:
            for i in range(self.num_assets):
                asset_yaw_index = i*self.num_ownship_states + 2
                state_vector[asset_yaw_index,0] = self._normalize_angle(state_vector[asset_yaw_index,0])
        elif self.world_dim == 3:
            # Assume x,y,z,roll,pitch,yaw, x_dot along base_link, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot
            for i in range(self.num_assets):
                asset_roll_index = i*self.num_ownship_states + 3
                asset_pitch_index = i*self.num_ownship_states + 4
                asset_yaw_index = i*self.num_ownship_states + 5
                state_vector[asset_roll_index,0] = self._normalize_angle(state_vector[asset_roll_index,0])
                state_vector[asset_pitch_index,0] = self._normalize_angle(state_vector[asset_pitch_index,0])
                state_vector[asset_yaw_index,0] = self._normalize_angle(state_vector[asset_yaw_index,0])
        return state_vector

    def _is_angle_meas(self, meas, check_implicit=False):
        if not check_implicit and isinstance(meas, Implicit):
            return False
        if isinstance(meas, GPSyaw_Explicit) or isinstance(meas, GPSyaw_Implicit):
            return True
        elif isinstance(meas, GPSyaw_Neighbor_Explicit) or isinstance(meas, GPSyaw_Neighbor_Implicit):
            return True
        elif isinstance(meas, Azimuth_Explicit) or isinstance(meas, Azimuth_Implicit):
            return True
        elif isinstance(meas, AzimuthGlobal_Explicit) or isinstance(meas, AzimuthGlobal_Implicit):
            return True
        else:
            return False

    def _linear_propagation(self, u):        
        # 1D not tracking velocity world
        if self.world_dim == 1 and self.num_ownship_states == 1:
            A = B = np.eye(1)
            self.x_hat = A*self.x_hat + B*u
            return A
        # 1D with velocity world
        elif self.world_dim == 1 and self.num_ownship_states == 2:
            A = np.eye(self.num_states)
            for i in range(self.num_assets):
                if i == self.my_id and self.is_main_fitler:
                    A[i*self.num_ownship_states+1,i*self.num_ownship_states+1] = 0 # set velocity to zero
                else:
                    A[i*self.num_ownship_states,i*self.num_ownship_states+1] = 1
            
            B = np.zeros((self.num_states,u.size))
            B[self.my_id*self.num_ownship_states,0] = 1
            B[self.my_id*self.num_ownship_states+1,0] = 1
            self.x_hat = np.dot(A,self.x_hat) + np.dot(B,u)
            return A
        # 2D with velocity tracking
        elif self.world_dim == 2 and self.num_ownship_states == 4:
            A = np.eye(self.num_states)
            for i in range(self.num_assets):
                if i == self.my_id and self.is_main_fitler:
                    A[i*self.num_ownship_states+2,i*self.num_ownship_states+2] = 0 # set velocity to zero
                    A[i*self.num_ownship_states+3,i*self.num_ownship_states+3] = 0 # set velocity to zero
                else:
                    A[i*self.num_ownship_states,i*self.num_ownship_states+2] = 1
                    A[i*self.num_ownship_states+1,i*self.num_ownship_states+3] = 1
            B = np.zeros((self.num_states,u.size))
            B[self.my_id*self.num_ownship_states,0] = 1
            B[self.my_id*self.num_ownship_states+1,0] = 1
            B[self.my_id*self.num_ownship_states+2,1] = 1
            B[self.my_id*self.num_ownship_states+3,1] = 1
            self.x_hat = np.dot(A,self.x_hat) + np.dot(B,u)
            return A
        else:
            raise NotImplementedError("Linear Propagation Not defined for state configuration. World dim: " \
                + str(self.world_dim) + " | Num Ownship States: " + str(self.num_ownship_states))

    def _nonlinear_propagation(self, u):
        ## Written for 2D
        if self.is_main_fitler:            
            self.x_hat[self.my_id * self.num_ownship_states + 3] = u[0,0] # speed
            self.x_hat[self.my_id * self.num_ownship_states + 5] = u[1,0] # angular velocity

        G = np.zeros((self.num_states, self.num_states))
        for a in range(self.num_assets):
            start_index = a*self.num_ownship_states
            s = self.x_hat[start_index + 3,0]
            theta_dot = self.x_hat[start_index + 5,0]
            # print("Propagating asset in estimate: " + str(a))
            # print("s: " + str(s))
            # print("theta_dot: " + str(theta_dot))

            theta_initial = self.x_hat[start_index+2,0]
            def dynamics(t, z):
                _x_dot = s * np.cos(z[2])
                _y_dot = s * np.sin(z[2])
                _theta_dot = theta_dot
                return np.array([_x_dot, _y_dot, _theta_dot])

            t_init, t_final = 0, 1
            z_init = self.x_hat[start_index:start_index + 3,0]
            r = integrate.RK45(dynamics, t_init, z_init, t_final)
            while r.status == "running":
                status = r.step()

            self.x_hat[start_index: start_index+3,0] = r.y
            
            # Construct this asset's part of jacobian
            G[start_index,start_index] = 1
            G[start_index + 1,start_index + 1] = 1
            G[start_index + 2, start_index + 2] = 1
            G[start_index + 3, start_index + 3] = 1
            G[start_index + 4, start_index + 4] = 1
            G[start_index + 5, start_index + 5] = 1
            G[start_index + 2, start_index + 5] = 1
            G[start_index, start_index + 2] = -s * np.sin(theta_initial + theta_dot/2)
            G[start_index + 1, start_index + 2] = s * np.cos(theta_initial + theta_dot/2)
            G[start_index, start_index + 3] = np.cos(theta_initial + theta_dot/2)
            G[start_index+1, start_index + 3] = np.sin(theta_initial + theta_dot/2)

            G[start_index, start_index + 5] = (-s * np.sin(theta_initial + theta_dot/2)) / 2
            G[start_index+1, start_index + 5] =  (s*np.cos(theta_initial + theta_dot/2)) / 2
            # if not self.is_main_fitler:
                

        # print("Output for asset " + str(self.my_id))
        # print(self.x_hat)
        return G

""" Main filter
differs slightly from an ETFilter in its implicit measurement update
If needs access to common filters for implicit measurement updates
"""
class ETFilter_Main( ETFilter ):
    def __init__(self, my_id, num_ownship_states, world_dim, x0, P0, predict_func, common_filters):
        """
        common_filters : dict
            key : int
                other asset id
            value : ETFiler
                common filter between both assets
        """
        super(ETFilter_Main, self).__init__(my_id, num_ownship_states, world_dim, x0, P0, predict_func)
        self.is_main_fitler = True
        self.common_filters = common_filters
    
    def _get_implicit_predata(self, C, R, x_hat_start, P_start, meas):
        x_ref = self._get_common_filter_states(meas.src_id).x_hat
        if meas.is_linear_meas:
            mu = C.dot(self.x_hat) - C.dot(x_hat_start )
            Qe = C.dot( P_start.dot( C.T )) + R
            alpha = C.dot( x_ref) - C.dot(x_hat_start )
        else: # Nonlinear Measurement
            mu0 = self._get_nonlinear_expected_meas(meas, self.x_hat) 
            mu1 = self._get_nonlinear_expected_meas(meas, x_hat_start)
            mu = mu0 - mu1
            Qe = np.abs(C.dot( P_start.dot( C.T )) + R)
            alpha0 = self._get_nonlinear_expected_meas(meas, x_ref)
            alpha1 = self._get_nonlinear_expected_meas(meas, x_hat_start)
            alpha = alpha0 - alpha1
        if self._is_angle_meas(meas, check_implicit=True):
            mu = self._normalize_angle(mu)
            alpha = self._normalize_angle(alpha)
        return mu, Qe, alpha

    def _get_common_filter_states(self, asset_id):
        return self.common_filters[asset_id]