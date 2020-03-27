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
            print(str(self.my_id) + " receiving meas: " + meas.__class__.__name__ + "| data: " + str(meas.data))

        if not isinstance(meas, Measurement):
            raise Exception("meas must of type Measurement")
        if self._is_angle_meas(meas):
            meas.data = self._normalize_angle(meas.data)
        
        self.meas_queue.append(meas)
    
    def predict(self, u, Q):
        if u.shape[1] > u.shape[0]:
            raise Exception("u must be column vector")
        if Q.shape != self.P.shape:
            raise Exception("Q must have state x state dimensions")

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
                innovation = self._get_innovation(meas, C)
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
        # Clear measurement queue
        self.meas_queue = []

    # If ETFilter_Main, method is overridden
    def _get_implicit_predata(self, C, R, x_hat_start, P_start, meas):
        mu = alpha = 0 # mu, alpha cancel out in the common information filter, not the case in main filter
        Qe = C.dot( P_start.dot( C.T )) + R
        return mu, Qe, alpha

    def _get_innovation(self, meas, C):
        if self._is_angle_meas(meas):
            return self._normalize_angle(meas.data - C.dot(self.x_hat))
        else:
            return meas.data - C.dot(self.x_hat)

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
        elif isinstance(meas, LinRelx_Explicit) or isinstance(meas, LinRelx_Implicit):
            C[0, src_id*self.num_ownship_states] = -1
            C[0, meas.measured_asset*self.num_ownship_states] = 1
        elif isinstance(meas, LinRely_Explicit) or isinstance(meas, LinRely_Implicit):
            C[0, src_id*self.num_ownship_states + 1] = -1
            C[0, meas.measured_asset*self.num_ownship_states + 1] = 1
        # elif isinstance(meas, LinRely_Explicit) or isinstance(meas, LinRely_Implicit):
        #     C = np.zeros((1, self.num_states))
        #     C[0, src_id*self.world_dim + 1] = -1
        #     C[0, meas.measured_asset*self.world_dim + 1] = 1
        else:
            raise NotImplementedError("Measurment Jacobian not implemented for: " + meas.__class__.__name__)
        return C
    
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
                if i == self.my_id:
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
                if i == self.my_id:
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

        self.x_hat[self.my_id * self.num_ownship_states + 3] = u[0,0] # speed
        self.x_hat[self.my_id * self.num_ownship_states + 5] = u[1,0] # angular velocity

        G = np.zeros((self.num_states, self.num_states))
        for a in range(self.num_assets):
            start_index = a*self.num_ownship_states
            s = self.x_hat[start_index + 3,0]
            theta_dot = self.x_hat[start_index + 5,0]

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
            G[start_index, start_index + 2] = -s * np.sin(theta_initial + theta_dot/2)
            G[start_index + 1, start_index + 2] = s * np.cos(theta_initial + theta_dot/2)
            G[start_index + 4, start_index + 4] = 1

            if a != my_id:
                G[start_index + 3, start_index + 3] = 1
                G[start_index + 5, start_index + 5] = 1

        # print("Output")
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
        self.common_filters = common_filters
    
    def _get_implicit_predata(self, C, R, x_hat_start, P_start, meas):
        x_ref = self._get_common_filter_states(meas.src_id).x_hat
        mu = C.dot( self.x_hat - x_hat_start )
        Qe = C.dot( P_start.dot( C.T )) + R
        alpha = C.dot( x_ref - x_hat_start )
        if self._is_angle_meas(meas, check_implicit=True):
            mu = self._normalize_angle(mu)
            alpha = self._normalize_angle(alpha)
        return mu, Qe, alpha

    def _get_common_filter_states(self, asset_id):
        return self.common_filters[asset_id]