from __future__ import division
import numpy as np
from measurements import *
from scipy.stats import norm as normal

# NOT THREADSAFE

class ETFilter(object):

    def __init__(self, my_id, x0, P0, A, B, et_delta):
        self.x_hat = x0
        self.P = P0
        self.A = A
        self.B = B
        self.et_delta = et_delta

        self.meas_queue = []

    def check_implicit(self, asset_id, meas):
        if not isinstance(meas, Measurement):
            print("[WARN] meas must of type Measurement")
            return

        data = meas.data

        if isinstance(meas, GPS_Explicit):
            C = np.array([[1]])
            expected_meas = C.dot( self.x_hat[asset_id] )
            return ( data - expected_meas ) <= self.et_delta

        elif isinstance(meas, LinRel_Explicit):
            print("This functionality is not ready yet!")

    def add_meas(self, asset_id, meas):
        if not isinstance(meas, Measurement):
            print("[WARN] meas must of type Measurement")
            return
        
        self.meas_queue.append([asset_id, meas])
    
    def predict(self, u, Q):
        self.x_hat = self.A.dot(self.x_hat) + self.B.dot(u)
        self.P = self.A.dot(self.P.dot( self.A.T )) + Q

    def correct(self):
        if not self.meas_queue:
            print("[WARN] meas_queue is empty!")
            return

        x_hat_start = self.x_hat
        P_start = self.P
        for [asset_id, meas] in self.meas_queue:
            K, C, R = None, None, None
            if isinstance(meas, Explicit):

                if isinstance(meas, GPS_Explicit):
                    # What would be super clean is if I have the measurement class
                    # calculate the jacobian then I map that jacobian to my whole state vector before using it
                    C = np.array([[1]]) # Move calculation of C to the individual measurmenet?
                    # TODO Use asset_id calculate the meas jacobian
                    # TODO properly map R
                    R = meas.R

                K = self._get_kalman_gain(C, R)
                innovation = self._get_innovation(meas.data, C)
                self.x_hat += K.dot(innovation)
                self.P = ( np.eye(self.x_hat.size) - K.dot(C) ).dot(self.P)
            else: # Implicit Update
                if isinstance(meas, GPS_Implicit):
                    C = np.array([[1]])
                    R = meas.R

                mu, Qe, alpha = self._get_implicit_predata(C, R, x_hat_start, P_start, asset_id)
                z_bar, curly_theta = self._get_implicit_data(mu, Qe, alpha)
                K = self._get_kalman_gain(C, R)
                self.x_hat += K.dot(z_bar)
                self.P += curly_theta * K.dot(self.P)
        # Clear measurement queue
        self.meas_queue = []

    # If ETFilter_Main, method is overridden
    def _get_implicit_predata(self, C, R, x_hat_start, P_start, asset_id):
        mu = alpha = np.zeros((self.x_hat.size, 1)) # mu, alpha always zero for a common filter
        Qe = C.dot( P_start.dot( C.T )) + R
        return mu, Qe, alpha

    def _get_innovation(self, meas, C):
        return meas - C.dot(self.x_hat)

    def _get_kalman_gain(self, C, R):
        tmp = C.dot(self.P.dot(C.T)) + R
        return self.P.dot(C.T).dot(np.linalg.inv( tmp ))

    def _get_implicit_data(self, mu, Qe, alpha):
        Q_func = lambda x : 1 - normal.cdf(x)
        
        arg1 = ( -self.delta + alpha - mu ) / np.sqrt( Qe )
        arg2 = ( self.delta + alpha - mu ) / np.sqrt( Qe )

        tmp = ( normal.pdf( arg1 ) - normal.pdf( arg2 ) ) / ( Q_func(arg1) - Q_func(arg2 ) )
        z_bar = tmp.dot( np.sqrt( Qe ) )

        # Check this matrix squaring, not elementwise
        tmp2 = ( arg1.dot( normal.pdf( arg1)) - arg2.dot( normal.pdf( arg2 )) ) / ( Q_func(arg1) - Q_func(arg2))
        curly_theta = np.linalg.matrix_power(tmp, 2) - tmp2

        return z_bar, curly_theta

""" Main filter
differs slightly from an ETFilter in its implicit measurement update
If needs access to common filters for implicit measurement updates
"""
class ETFilter_Main( ETFilter ):
    def __init__(self, my_id, x0, P0, A, B, et_delta, common_filters):
        """
        common_filters : dict
            key : int
                other asset id
            value : ETFiler
                common filter between both assets
        """
        super(ETFilter_Main, self).__init__(my_id, x0, P0, A, B, et_delta)
        self.common_filters = common_filters
    
    def _get_implicit_predata(self, C, R, x_hat_start, P_start, asset_id):
        x_ref = self._get_common_filter_states(asset_id)
        mu = C.dot( self.x_hat - x_hat_start )
        Qe = C.dot( P_start.dot( C.T )) + R
        alpha = C.dot( x_ref - x_hat_start )
        return mu, Qe, alpha

    def _get_common_filter_states(self, asset_id):
        return self.common_filters[asset_id]