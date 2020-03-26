from __future__ import division
import numpy as np
from measurements import *
from scipy.stats import norm as normal
from copy import deepcopy
from pdb import set_trace

class ETFilter(object):

    def __init__(self, my_id, num_ownship_states, world_dim, x0, P0, predict_func):
        self.my_id = my_id
        self.num_ownship_states = num_ownship_states
        self.world_dim = world_dim
        self.x_hat = x0
        self.P = P0
        self.predict_func = predict_func
        self.num_states = self.x_hat.size
        if (self.num_states % num_ownship_states) != 0:
            raise Exception("Dimensionality of state vector does not align with the number of ownship states.")
        elif num_ownship_states < world_dim:
            raise Exception("Number of ownship states does not make sense for world dimension")
        self.num_assets = int( self.num_states / num_ownship_states )
        
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
        if not isinstance(meas, Measurement):
            raise Exception("meas must of type Measurement")
        if self._is_angle_meas(meas):
            meas.data = self._normalize_angle(meas.data)
        
        self.meas_queue.append(meas)
    
    def predict(self, u, Q):
        (self.x_hat, G) = self.predict_func( deepcopy(self.x_hat) , deepcopy(u), self.my_id) # Pass by value to function pointers
        self.x_hat = self._normalize_all_angles(self.x_hat)

        self.P = G.dot( self.P.dot( G.T )) + Q

    def correct(self):
        if not self.meas_queue:
            # print("meas_queue is empty!")
            return
        # print("Msg queue:")
        # for meas in self.meas_queue:
        #     print(meas.data)

        x_hat_start = self.x_hat
        P_start = self.P
        for meas in self.meas_queue:
            K, C = None, None
            R = meas.R
            if isinstance(meas, Explicit):
                C = self._get_measurement_jacobian(meas)
                K = self._get_kalman_gain(C, R)
                innovation = self._get_innovation(meas, C)
                self.x_hat += K.dot(innovation)
                self.P = ( np.eye(self.num_states) - K.dot(C) ).dot(self.P)
            else: # Implicit Update
                C = self._get_measurement_jacobian(meas)
                mu, Qe, alpha = self._get_implicit_predata(C, R, x_hat_start, P_start, meas.src_id)
                z_bar, curly_theta = self._get_implicit_data(meas.et_delta, mu, Qe, alpha)
                K = self._get_kalman_gain(C, R)
                if self._is_angle_meas(meas):
                    z_bar = self._normalize_angle(z_bar)
                self.x_hat += K.dot(z_bar)
                self.P = self.P - curly_theta * K.dot(C.dot(self.P))
            self.x_hat = self._normalize_all_angles( self.x_hat )
        # Clear measurement queue
        self.meas_queue = []

    # If ETFilter_Main, method is overridden
    def _get_implicit_predata(self, C, R, x_hat_start, P_start, asset_id):
        mu = alpha = 0 # mu, alpha cancel out in the common information filter, not the case in main filter
        Qe = C.dot( P_start.dot( C.T )) + R
        return mu, Qe, alpha

    def _get_innovation(self, meas, C):
        if self._is_angle_meas(meas):
            return self._normalize_angle(meas.data - C.dot(self.x_hat))
        return meas.data - C.dot(self.x_hat)

    def _get_kalman_gain(self, C, R):
        tmp = np.dot( np.dot(C, self.P), C.T ) + R
        return self.P.dot(C.T.dot( np.linalg.inv( tmp ) ))

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

    def _is_angle_meas(self, meas):
        if isinstance(meas, GPSyaw_Explicit) or isinstance(meas, GPSyaw_Implicit):
            return True
        else:
            return False

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
    
    def _get_implicit_predata(self, C, R, x_hat_start, P_start, asset_id):
        x_ref = self._get_common_filter_states(asset_id).x_hat
        mu = C.dot( self.x_hat - x_hat_start )
        Qe = C.dot( P_start.dot( C.T )) + R
        alpha = C.dot( x_ref - x_hat_start )
        return mu, Qe, alpha

    def _get_common_filter_states(self, asset_id):
        return self.common_filters[asset_id]