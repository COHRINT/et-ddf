from __future__ import division
import numpy as np
from measurements import *
from scipy.stats import norm as normal

class ETFilter(object):

    def __init__(self, my_id, num_states_per_asset, x0, P0, A, B, et_delta):
        self.my_id = my_id
        self.dim = num_states_per_asset
        self.x_hat = x0
        self.P = P0
        self.A = A
        self.B = B
        self.et_delta = et_delta
        self.num_states = self.x_hat.size
        if (self.num_states % num_states_per_asset) != 0:
            raise Exception("Dimensionality of state vector does not align with the number states per asset.")
        
        self.meas_queue = []

    def check_implicit(self, asset_id, meas):
        if not isinstance(meas, Measurement):
            raise Exception("meas must of type Measurement")

        C = self._get_measurement_jacobian(asset_id, meas)
        innovation = self._get_innovation(meas.data, C)
        return np.abs(innovation) <= self.et_delta

    def add_meas(self, asset_id, meas):
        if not isinstance(meas, Measurement):
            raise Exception("meas must of type Measurement")
        
        self.meas_queue.append([asset_id, meas])
    
    def predict(self, u, Q):
        self.x_hat = self.A.dot(self.x_hat) + self.B.dot(u)
        self.P = self.A.dot(self.P.dot( self.A.T )) + Q

    def correct(self):
        if not self.meas_queue:
            # print("meas_queue is empty!")
            return

        x_hat_start = self.x_hat
        P_start = self.P
        for [asset_id, meas] in self.meas_queue:
            K, C = None, None
            R = meas.R
            if isinstance(meas, Explicit):
                C = self._get_measurement_jacobian(asset_id, meas)
                K = self._get_kalman_gain(C, R)
                innovation = self._get_innovation(meas.data, C)
                self.x_hat += K.dot(innovation)
                self.P = ( np.eye(self.num_states) - K.dot(C) ).dot(self.P)
            else: # Implicit Update
                C = self._get_measurement_jacobian(asset_id, meas)
                mu, Qe, alpha = self._get_implicit_predata(C, R, x_hat_start, P_start, asset_id)
                z_bar, curly_theta = self._get_implicit_data(mu, Qe, alpha)
                K = self._get_kalman_gain(C, R)
                self.x_hat += K.dot(z_bar)
                self.P = self.P - curly_theta * K.dot(C.dot(self.P))
        # Clear measurement queue
        self.meas_queue = []

    # If ETFilter_Main, method is overridden
    def _get_implicit_predata(self, C, R, x_hat_start, P_start, asset_id):
        mu = alpha = 0 # mu, alpha cancel out in the common information filter, not the case in main filter
        Qe = C.dot( P_start.dot( C.T )) + R
        return mu, Qe, alpha

    def _get_innovation(self, meas, C):
        return meas - C.dot(self.x_hat)

    def _get_kalman_gain(self, C, R):
        tmp = np.dot( np.dot(C, self.P), C.T ) + R
        return self.P.dot(C.T.dot( np.linalg.inv( tmp ) ))

    def _get_implicit_data(self, mu, Qe, alpha):
        Q_func = lambda x : 1 - normal.cdf(x)
        
        arg1 = ( -self.et_delta + alpha - mu ) / np.sqrt( Qe )
        arg2 = ( self.et_delta + alpha - mu ) / np.sqrt( Qe )

        tmp = ( normal.pdf( arg1 ) - normal.pdf( arg2 ) ) / ( Q_func(arg1) - Q_func(arg2 ) )
        z_bar = tmp.dot( np.sqrt( Qe ) )

        tmp2 = ( arg1.dot( normal.pdf( arg1)) - arg2.dot( normal.pdf( arg2 )) ) / ( Q_func(arg1) - Q_func(arg2))
        curly_theta = np.linalg.matrix_power(tmp, 2) - tmp2

        return z_bar, curly_theta

    def _get_measurement_jacobian(self, asset_id, meas):
        C = None
        if isinstance(meas, GPSx_Explicit) or isinstance(meas, GPSx_Implicit):
            C = np.zeros((1, self.num_states))
            C[0, asset_id*self.dim] = 1
        elif isinstance(meas, GPSy_Explicit) or isinstance(meas, GPSy_Implicit):
            C = np.zeros((1, self.num_states))
            C[0, asset_id*self.dim + 1] = 1
        elif isinstance(meas, LinRelx_Explicit) or isinstance(meas, LinRelx_Implicit):
            C = np.zeros((1, self.num_states))
            C[0, asset_id*self.dim] = -1
            C[0, meas.other_asset*self.dim] = 1
        else:
            raise NotImplementedError("Measurment Jacobian not implemented for: " + meas.__class__.__name__)
        return C

""" Main filter
differs slightly from an ETFilter in its implicit measurement update
If needs access to common filters for implicit measurement updates
"""
class ETFilter_Main( ETFilter ):
    def __init__(self, my_id, num_states_per_asset, x0, P0, A, B, et_delta, common_filters):
        """
        common_filters : dict
            key : int
                other asset id
            value : ETFiler
                common filter between both assets
        """
        super(ETFilter_Main, self).__init__(my_id, num_states_per_asset, x0, P0, A, B, et_delta)
        self.common_filters = common_filters
    
    def _get_implicit_predata(self, C, R, x_hat_start, P_start, asset_id):
        x_ref = self._get_common_filter_states(asset_id).x_hat
        mu = C.dot( self.x_hat - x_hat_start )
        Qe = C.dot( P_start.dot( C.T )) + R
        alpha = C.dot( x_ref - x_hat_start )
        return mu, Qe, alpha

    def _get_common_filter_states(self, asset_id):
        return self.common_filters[asset_id]