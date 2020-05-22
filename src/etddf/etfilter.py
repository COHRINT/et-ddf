from __future__ import division
"""@package etddf

Filter class for event triggering.

Primary filter class is ETFilter but an asset's main filter differs slightly. The main filter needs access
to common filter's (ETFilter) estimate in order to fuse implicit measurements. Therefore another class exists,
ETFilter_Main, that also takes in a dictionary with common filters to other assets.

Supports following dynamics, dimensions and states
 - Linear 1D (x, x_dot)
 - Linear 2D (x, y, x_dot, y_dot)
 - Nonlinear 2D (x, y, yaw, x_dot, y_dot, yaw_dot)
 - Linear 3D (x, y, z, x_dot, y_dot, z_dot)
 - Nonlinear 3D (x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot)

Supports fusing all Explicit and Implicit measurements defined in etddf/measurements.py
"""
__author__ = "Luke Barbier"
__copyright__ = "Copyright 2020, COHRINT Lab"
__email__ = "luke.barbier@colorado.edu"
__status__ = "Development"
__license__ = "MIT"
__maintainer__ = "Luke Barbier"
__version__ = "1.1.0"

import numpy as np
from etddf.measurements import *
from etddf.dynamics import linear_propagation
from etddf.normalize_angle import *
from etddf.measurement_expected import get_nonlinear_expected_meas
from etddf.measurement_jacobians import get_measurement_jacobian
from etddf.instantiate_asset import instantiate_asset_linrel, check_instantiate_asset_linrel
from scipy.stats import norm as normal
from copy import deepcopy
from pdb import set_trace
from scipy import integrate

DEBUG=False
# Uncertainty hreshold at which we drop all previous tracking and instantiate a new tracking of an asset based off of range/bearing measurement data
# Used on first measurement of red asset
NO_ASSET_INFORMATION = 50**2 

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
            raise ValueError("Dimensionality of state vector does not align with the number of ownship states.")
        elif num_ownship_states < world_dim:
            raise ValueError("Number of ownship states does not make sense for world dimension")
        self.num_assets = int( self.num_states / self.num_ownship_states )
        self.meas_queue = []

    def check_implicit(self, meas):
        """Checks if a measurement can be fused implicitly

        Arguments:
            meas {etddf.measurements.Explicit} -- Measurement to be checked

        Raises:
            TypeError: meas is not of type Explicit

        Returns:
            bool -- True for implicit / False for explicit
        """
        # TODO add support for scaling innovation check by the uncertainty
        if not isinstance(meas, Explicit):
            raise TypeError("meas must of type Explicit")
        if meas.is_angle_meas:
            meas.data = normalize_angle(meas.data)

        C = get_measurement_jacobian(meas, self.x_hat, self.num_states, self.world_dim, self.num_ownship_states)
        innovation = self._get_innovation(meas, C)
        return np.abs(innovation) <= meas.et_delta

    def add_meas(self, meas):
        """Adds a measurement to the filter

        Arguments:
            meas {etddf.measurements.Measurement} -- Measurement to be fused

        Raises:
            TypeError: meas is not of type Measurement
        """
        if DEBUG:
            print(str(self.my_id) + " receiving meas: " + meas.__class__.__name__ + " | data: " + str(meas.data))

        if not isinstance(meas, Measurement):
            raise TypeError("meas must of type Measurement")
        if meas.is_angle_meas and not isinstance(meas, Implicit):
            meas.data = normalize_angle(meas.data)

        self.meas_queue.append(meas)

        # Check if we should instantiate this asset based off of the meas LinRel information
        if check_instantiate_asset_linrel(meas, self.P, self.meas_queue, NO_ASSET_INFORMATION, self.num_ownship_states):
            self.x_hat, self.P, self.meas_queue = instantiate_asset_linrel(meas.measured_asset_id, deepcopy(self.x_hat), deepcopy(self.P), deepcopy(self.meas_queue), self.num_ownship_states)
        
    def predict(self, u, Q, time_delta=1.0, use_control_input=False):
        """Runs predicion step on the filter

        Arguments:
            u {np.ndarray} -- control input (num_ownship_states / 2, 1)
            Q {np.ndarray} -- motion/process noise (nstates, nstates)

        Keyword Arguments:
            time_delta {float} -- Amount of time to predict in future (default: {1.0})
            use_control_input {bool} -- Whether to use control input or assume constant velocity (default: {False})

        Raises:
            ValueError: Incorrect u dimensions
            ValueError: Incorrect Q dimensions
            NotImplementedError: Nonlinear Propagation
        """
        if u.shape[1] > u.shape[0]:
            raise ValueError("u must be column vector")
        if Q.shape != self.P.shape:
            raise ValueError("Q must have (state x state) dimensions")

        if self.linear_dynamics:
            self.x_hat, A = linear_propagation(self.x_hat, u, self.num_ownship_states, self.my_id, time_delta, use_control_input)
            self.P = A.dot( self.P.dot( A.T )) + Q
        else: # nonlinear dynamics
            # G = nonlinear_propagation(u)
            # self.P = G.dot( self.P.dot( G.T )) + Q
            # self.x_hat = normalize_all_angles(self.x_hat, self.num_ownship_states, self.num_assets, self.world_dim)
            raise NotImplementedError("nonlinear propagation requested")

    def correct(self):
        """Runs correction step on the filter
        """
        if not self.meas_queue:
            # print("meas_queue is empty!")
            return

        x_hat_start = deepcopy(self.x_hat)
        P_start = deepcopy(self.P)
        for meas in self.meas_queue:
            if DEBUG:
                print("Fusing " + meas.__class__.__name__ + " of " + str(meas.src_id) + " w/ data: " + str(meas.data))
                print("State of Filter:")
                print("x_hat")
                print(self.x_hat)
                print("P")
                print(self.P)

            R = meas.R
            if isinstance(meas, Explicit):
                C = get_measurement_jacobian(meas, self.x_hat, self.num_states, self.world_dim, self.num_ownship_states)
                K = self._get_kalman_gain(C, R)
                innovation = self._get_innovation(meas, C).reshape(1,1)
                self.x_hat += np.dot( K, innovation)
                tmp = np.eye(self.num_states) - np.dot(K, C)
                self.P = tmp.dot(self.P)
                # self.P = np.dot( np.dot(tmp, self.P), tmp.T) + K.dot(R_mat.dot(K.T))
            else: # Implicit Update
                C = get_measurement_jacobian(meas, self.x_hat, self.num_states, self.world_dim, self.num_ownship_states)
                mu, Qe, alpha = self._get_implicit_predata(C, R, x_hat_start, P_start, meas)
                z_bar, curly_theta = self._get_implicit_data(meas.et_delta, mu, Qe, alpha)
                K = self._get_kalman_gain(C, R)
                if meas.is_angle_meas:
                    z_bar = normalize_angle(z_bar)
                self.x_hat += np.dot(K, z_bar)
                self.P = self.P - curly_theta * K.dot(C.dot(self.P))
            self.x_hat = normalize_all_angles( self.x_hat , self.num_ownship_states, self.num_assets, self.world_dim)
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
            expected_meas = get_nonlinear_expected_meas(meas, self.x_hat, self.world_dim, self.num_ownship_states)
        if DEBUG:
            print("expected meas " + meas.__class__.__name__ + " : " + str(expected_meas))
            print("---> actual meas: " + str(meas.data))
        if meas.is_angle_meas:
            return normalize_angle( meas.data - expected_meas)
        else:
            return meas.data - expected_meas

    def _get_kalman_gain(self, C, R):
        tmp = np.dot( np.dot(C, self.P), C.T ) + R
        tmp_inv = np.linalg.inv( tmp ) if tmp.size > 1 else tmp**(-1) # Accomadate 1D and >1D filter
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

class ETFilter_Main( ETFilter ):
    """Main filter for an asset
    Differs slightly from an ETFilter in its implicit measurement update because it 
    needs access to common filters with other assets to fuse implicit measurements
    """
    def __init__(self, my_id, num_ownship_states, world_dim, x0, P0, linear_dynamics, common_filters):
        """
        common_filters : dict
            key : int
                other asset id
            value : ETFiler
                common filter between both assets
        """
        super(ETFilter_Main, self).__init__(my_id, num_ownship_states, world_dim, x0, P0, linear_dynamics)
        self.common_filters = common_filters
    
    def _get_implicit_predata(self, C, R, x_hat_start, P_start, meas):
        x_ref = self._get_common_filter_states(meas.src_id).x_hat
        if meas.is_linear_meas:
            mu = C.dot(self.x_hat) - C.dot(x_hat_start )
            Qe = np.abs(C.dot( P_start.dot( C.T )) + R)
            alpha = C.dot( x_ref) - C.dot(x_hat_start )
        else: # Nonlinear Measurement
            mu0 = get_nonlinear_expected_meas(meas, self.x_hat, self.world_dim, self.num_ownship_states) 
            mu1 = get_nonlinear_expected_meas(meas, x_hat_start, self.world_dim, self.num_ownship_states)
            mu = mu0 - mu1
            Qe = np.abs(C.dot( P_start.dot( C.T )) + R)
            alpha0 = get_nonlinear_expected_meas(meas, x_ref, self.world_dim, self.num_ownship_states)
            alpha1 = get_nonlinear_expected_meas(meas, x_hat_start, self.world_dim, self.num_ownship_states)
            alpha = alpha0 - alpha1
        if meas.is_angle_meas:
            mu = normalize_angle(mu)
            alpha = normalize_angle(alpha)
        return mu, Qe, alpha

    def _get_common_filter_states(self, asset_id):
        # If only one common filter, just return that one
        if len(self.common_filters) == 1:
            return self.common_filters[self.common_filters.keys()[0]]
        # return the specific common filter with that asset
        return self.common_filters[asset_id]