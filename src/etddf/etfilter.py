from __future__ import division
import numpy as np
from etddf.measurements import *
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
            raise Exception("Dimensionality of state vector does not align with the number of ownship states.")
        elif num_ownship_states < world_dim:
            raise Exception("Number of ownship states does not make sense for world dimension")
        self.num_assets = int( self.num_states / self.num_ownship_states )
        self.is_main_fitler = False
        self.meas_queue = []

    def check_implicit(self, meas):
        if not isinstance(meas, Measurement):
            raise Exception("meas must of type Measurement")
        if meas.is_angle_meas:
            meas.data = self._normalize_angle(meas.data)

        C = self._get_measurement_jacobian(meas)
        innovation = self._get_innovation(meas, C)
        return np.abs(innovation) <= meas.et_delta

    def add_meas(self, meas):
        if DEBUG:
            print(str(self.my_id) + " receiving meas: " + meas.__class__.__name__ + " | data: " + str(meas.data))

        if not isinstance(meas, Measurement):
            raise Exception("meas must of type Measurement")
        if meas.is_angle_meas and not isinstance(meas, Implicit):
            meas.data = self._normalize_angle(meas.data)

        self.meas_queue.append(meas)

        # Check if this is our first meas of asset
        # If the meas is a range/bearing, instantiate the asset at measurement mean
        if isinstance(meas, Azimuth_Explicit) or isinstance(meas, Range_Explicit) or isinstance(meas, Elevation_Explicit):
            if self.P[meas.measured_asset*self.num_ownship_states, meas.measured_asset*self.num_ownship_states] > NO_ASSET_INFORMATION:
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

            K, C = None, None
            R = meas.R
            if isinstance(meas, Explicit):
                C = self._get_measurement_jacobian(meas)
                K = self._get_kalman_gain(C, R)
                innovation = self._get_innovation(meas, C).reshape(1,1)                
                self.x_hat += np.dot( K, innovation)
                # TODO replace with joseph form of measurement update
                self.P = ( np.eye(self.num_states) - np.dot(K, C) ).dot(self.P)
            else: # Implicit Update
                C = self._get_measurement_jacobian(meas)
                mu, Qe, alpha = self._get_implicit_predata(C, R, x_hat_start, P_start, meas)
                z_bar, curly_theta = self._get_implicit_data(meas.et_delta, mu, Qe, alpha)
                K = self._get_kalman_gain(C, R)
                if meas.is_angle_meas:
                    z_bar = self._normalize_angle(z_bar)
                self.x_hat += np.dot(K, z_bar)
                self.P = self.P - curly_theta * K.dot(C.dot(self.P))
            self.x_hat = self._normalize_all_angles( self.x_hat )
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
        if meas.is_angle_meas:
            return self._normalize_angle( meas.data - expected_meas)
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

    def _get_measurement_jacobian(self, meas):
        C = np.zeros((1, self.num_states))
        src_id = meas.src_id
        if isinstance(meas, GPSx_Explicit) or isinstance(meas, GPSx_Implicit):
            C[0, src_id*self.num_ownship_states] = 1
        elif isinstance(meas, GPSy_Explicit) or isinstance(meas, GPSy_Implicit):
            C[0, src_id*self.num_ownship_states + 1] = 1
        elif isinstance(meas, GPSz_Explicit) or isinstance(meas, GPSz_Implicit):
            C[0, src_id*self.num_ownship_states + 2] = 1
        elif isinstance(meas, GPSyaw_Explicit) or isinstance(meas, GPSyaw_Implicit):
            if self.world_dim == 2:
                C[0, src_id*self.num_ownship_states + 2] = 1
            else: # world dim 3
                C[0, src_id*self.num_ownship_states + 3] = 1
        elif isinstance(meas, GPSx_Neighbor_Explicit) or isinstance(meas, GPSx_Neighbor_Implicit):
            C[0, meas.neighbor_id*self.num_ownship_states] = 1
        elif isinstance(meas, GPSy_Neighbor_Explicit) or isinstance(meas, GPSy_Neighbor_Implicit):
            C[0, meas.neighbor_id*self.num_ownship_states+1] = 1
        elif isinstance(meas, GPSz_Neighbor_Explicit) or isinstance(meas, GPSz_Neighbor_Implicit):
            C[0, meas.neighbor_id*self.num_ownship_states+2] = 1
        elif isinstance(meas, GPSyaw_Neighbor_Explicit) or isinstance(meas, GPSyaw_Neighbor_Implicit):
            if self.world_dim == 2:
                C[0, meas.neighbor_id*self.num_ownship_states + 2] = 1
            else: # world dim 3
                C[0, meas.neighbor_id*self.num_ownship_states + 3] = 1
        elif isinstance(meas, Azimuth_Explicit) or isinstance(meas, Azimuth_Implicit):
            meas_id = meas.measured_asset
            src_x = self.x_hat[src_id*self.num_ownship_states,0]
            src_y = self.x_hat[src_id*self.num_ownship_states+1,0]
            other_x = self.x_hat[meas_id*self.num_ownship_states,0]
            other_y = self.x_hat[meas_id*self.num_ownship_states+1,0]
            diff_x = other_x - src_x
            diff_y = other_y - src_y

            # Protect division by zero
            diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
            diff_y = diff_y if abs(diff_y) > 0.01 else 0.01

            # Azimuth jacobians
            C[0, src_id*self.num_ownship_states] = diff_y / ( diff_x**2 + diff_y**2 )
            C[0, meas_id*self.num_ownship_states] = -diff_y / ( diff_x**2 + diff_y**2 )
            C[0, src_id*self.num_ownship_states+1] = -diff_x / ( diff_x**2 + diff_y**2 )
            C[0, meas_id*self.num_ownship_states+1] = diff_x / ( diff_x**2 + diff_y**2 )
            # d_az/d_theta is in a different index depending on 2D or 3D
            if self.world_dim == 2:
                C[0, src_id*self.num_ownship_states + 2] = -1
            else: # 3D World
                C[0, src_id*self.num_ownship_states + 3] = -1
        elif isinstance(meas, AzimuthGlobal_Explicit) or isinstance(meas, AzimuthGlobal_Implicit):
            src_x = self.x_hat[src_id*self.num_ownship_states,0]
            src_y = self.x_hat[src_id*self.num_ownship_states+1,0]
            other_x = meas.global_pos[0]
            other_y = meas.global_pos[1]
            diff_x = other_x - src_x
            diff_y = other_y - src_y

            # Protect division by zero
            diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
            diff_y = diff_y if abs(diff_y) > 0.01 else 0.01

            # Azimuth jacobians
            C[0, src_id*self.num_ownship_states] = diff_y / ( diff_x**2 + diff_y**2 )
            C[0, src_id*self.num_ownship_states+1] = -diff_x / ( diff_x**2 + diff_y**2 )
            # d_az/d_theta is in a different index depending on 2D or 3D
            if self.world_dim == 2:
                C[0, src_id*self.num_ownship_states + 2] = -1
            else: # 3D World
                C[0, src_id*self.num_ownship_states + 3] = -1
        elif isinstance(meas, Range_Explicit) or isinstance(meas, Range_Implicit):
            meas_id = meas.measured_asset
            src_x = self.x_hat[src_id*self.num_ownship_states,0]
            src_y = self.x_hat[src_id*self.num_ownship_states+1,0]
            other_x = self.x_hat[meas_id*self.num_ownship_states,0]
            other_y = self.x_hat[meas_id*self.num_ownship_states+1,0]
            diff_x = other_x - src_x
            diff_y = other_y - src_y
            
            if self.world_dim == 2:
                r = np.sqrt( diff_x**2 + diff_y**2 )
                r = r if r > 0.01 else 0.01 # Division by zero protection
                C[0, src_id*self.num_ownship_states] = -diff_x / r
                C[0, meas_id*self.num_ownship_states] = diff_x / r
                C[0, src_id*self.num_ownship_states+1] = -diff_y / r
                C[0, meas_id*self.num_ownship_states+1] = diff_y / r
            else: # World Dim 3D
                src_z = self.x_hat[src_id*self.num_ownship_states+2,0]
                other_z = self.x_hat[meas_id*self.num_ownship_states+2,0]
                diff_z = other_z - src_z
                r = np.sqrt( diff_x**2 + diff_y**2 + diff_z**2 )
                r = r if r > 0.01 else 0.01 # Division by zero protection
                C[0, src_id*self.num_ownship_states] = -diff_x / r
                C[0, meas_id*self.num_ownship_states] = diff_x / r
                C[0, src_id*self.num_ownship_states+1] = -diff_y / r
                C[0, meas_id*self.num_ownship_states+1] = diff_y / r
                C[0, src_id*self.num_ownship_states+2] = -diff_z / r
                C[0, meas_id*self.num_ownship_states+2] = diff_z / r

        elif isinstance(meas, RangeGlobal_Explicit) or isinstance(meas, RangeGlobal_Implicit):
            src_x = self.x_hat[src_id*self.num_ownship_states,0]
            src_y = self.x_hat[src_id*self.num_ownship_states+1,0]
            other_x = meas.global_pos[0]
            other_y = meas.global_pos[1]
            diff_x = other_x - src_x
            diff_y = other_y - src_y
            
            if self.world_dim == 2:
                r = np.sqrt( diff_x**2 + diff_y**2 )
                r = r if r > 0.01 else 0.01 # Division by zero protection
                C[0, src_id*self.num_ownship_states] = -diff_x / r
                C[0, src_id*self.num_ownship_states+1] = -diff_y / r
            else: # World Dim 3D
                src_z = self.x_hat[src_id*self.num_ownship_states+2,0]
                other_z = meas.global_pos[2]
                diff_z = other_z - src_z
                r = np.sqrt( diff_x**2 + diff_y**2 + diff_z**2 )
                r = r if r > 0.01 else 0.01 # Division by zero protection
                C[0, src_id*self.num_ownship_states] = -diff_x / r
                C[0, src_id*self.num_ownship_states+1] = -diff_y / r
                C[0, src_id*self.num_ownship_states+2] = -diff_z / r

        elif isinstance(meas, Elevation_Explicit) or isinstance(meas, Elevation_Implicit):
            meas_id = meas.measured_asset
            src_x = self.x_hat[meas.src_id*self.num_ownship_states,0]
            src_y = self.x_hat[meas.src_id*self.num_ownship_states+1,0]
            src_z = self.x_hat[meas.src_id*self.num_ownship_states+2,0]
            other_x = self.x_hat[meas_id*self.num_ownship_states,0]
            other_y = self.x_hat[meas_id*self.num_ownship_states+1,0]
            other_z = self.x_hat[meas_id*self.num_ownship_states+2,0]
            diff_x = other_x - src_x
            diff_y = other_y - src_y
            diff_z = other_z - src_z

            all_diff = diff_x**2 + diff_y**2 + diff_z**2
            all_diff = all_diff if all_diff > 0.01 else 0.01 # Division by zero protection
            
            ## Own asset part of jacobian
            # d_el / dx_src
            C[0, src_id*self.num_ownship_states] = (diff_z*diff_x) / (np.sqrt(1-(diff_z**2)/all_diff) * np.power(all_diff, 3/2) )
            # d_el / dy_src
            C[0, src_id*self.num_ownship_states+1] = (diff_z*diff_y) / (np.sqrt(1-(diff_z**2)/all_diff) * np.power(all_diff, 3/2) )
            # d_el / dz_src
            C[0, src_id*self.num_ownship_states+2] = - np.sqrt(diff_x**2 + diff_y**2) / all_diff

            ## Other Asset part of jacobian
            # d_el / dx_other
            C[0, meas_id*self.num_ownship_states] = -C[0, src_id*self.num_ownship_states]
            # d_el / dy_other
            C[0, meas_id*self.num_ownship_states+1] = -C[0, src_id*self.num_ownship_states]
            # d_el / dz_other
            C[0, meas_id*self.num_ownship_states+2] = -C[0, src_id*self.num_ownship_states+2]
        elif isinstance(meas, ElevationGlobal_Explicit) or isinstance(meas, ElevationGlobal_Implicit):
            src_x = self.x_hat[meas.src_id*self.num_ownship_states,0]
            src_y = self.x_hat[meas.src_id*self.num_ownship_states+1,0]
            src_z = self.x_hat[meas.src_id*self.num_ownship_states+2,0]
            other_x = meas.global_pos[0]
            other_y = meas.global_pos[1]
            other_z = meas.global_pos[2]
            diff_x = other_x - src_x
            diff_y = other_y - src_y
            diff_z = other_z - src_z

            all_diff = diff_x**2 + diff_y**2 + diff_z**2
            
            ## Own asset
            # d_el / dx_src
            C[0, src_id*self.num_ownship_states] = (diff_z*diff_x) / (np.sqrt(1-(diff_z**2)/all_diff) * np.power(all_diff, 3/2) )
            # d_el / dy_src
            C[0, src_id*self.num_ownship_states+1] = (diff_z*diff_y) / (np.sqrt(1-(diff_z**2)/all_diff) * np.power(all_diff, 3/2) )
            # d_el / dz_src
            C[0, src_id*self.num_ownship_states+2] = - np.sqrt(diff_x**2 + diff_y**2) / all_diff
        else:
            raise NotImplementedError("Measurment Jacobian not implemented for: " + meas.__class__.__name__)
        return C

    def _get_nonlinear_expected_meas(self, meas, x_hat):
        if isinstance(meas, Azimuth_Explicit) or isinstance(meas, Azimuth_Implicit):
            if self.world_dim == 2:
                src_bearing = x_hat[meas.src_id*self.num_ownship_states + 2,0]
            else:
                src_bearing = x_hat[meas.src_id*self.num_ownship_states + 3,0]
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
            else:
                src_bearing = x_hat[meas.src_id*self.num_ownship_states + 3,0]
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
            src_x = x_hat[meas.src_id*self.num_ownship_states,0]
            src_y = x_hat[meas.src_id*self.num_ownship_states+1,0]
            other_x = x_hat[meas.measured_asset*self.num_ownship_states,0]
            other_y = x_hat[meas.measured_asset*self.num_ownship_states+1,0]
            diff_x = other_x - src_x
            diff_y = other_y - src_y
            if self.world_dim == 2:
                return np.sqrt( diff_x**2 + diff_y**2 )
            else: # 3D World
                src_z = x_hat[meas.src_id*self.num_ownship_states+2,0]
                other_z = x_hat[meas.measured_asset*self.num_ownship_states+2,0]
                diff_z = other_z - src_z
                return np.sqrt( diff_x**2 + diff_y**2 + diff_z**2 )
                
        elif isinstance(meas, RangeGlobal_Explicit) or isinstance(meas, RangeGlobal_Implicit):
            src_x = x_hat[meas.src_id*self.num_ownship_states,0]
            src_y = x_hat[meas.src_id*self.num_ownship_states+1,0]
            other_x = meas.global_pos[0]
            other_y = meas.global_pos[1]
            diff_x = other_x - src_x
            diff_y = other_y - src_y
            if self.world_dim == 2:
                return np.sqrt( diff_x**2 + diff_y**2 )
            else: # 3D World
                src_z = x_hat[meas.src_id*self.num_ownship_states+2,0]
                other_z = meas.global_pos[2]
                diff_z = other_z - src_z
                return np.sqrt( diff_x**2 + diff_y**2 + diff_z**2 )
        elif isinstance(meas, Elevation_Explicit) or isinstance(meas, Elevation_Implicit):
            src_x = x_hat[meas.src_id*self.num_ownship_states,0]
            src_y = x_hat[meas.src_id*self.num_ownship_states+1,0]
            src_z = x_hat[meas.src_id*self.num_ownship_states+2,0]
            other_x = x_hat[meas.measured_asset*self.num_ownship_states,0]
            other_y = x_hat[meas.measured_asset*self.num_ownship_states+1,0]
            other_z = x_hat[meas.measured_asset*self.num_ownship_states+2,0]
            diff_x = other_x - src_x
            diff_y = other_y - src_y
            diff_z = other_z - src_z

            # Protect division by zero
            diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
            diff_y = diff_y if abs(diff_y) > 0.01 else 0.01
            diff_z = diff_z if abs(diff_z) > 0.01 else 0.01

            expected_elevation = np.arcsin(diff_z / np.linalg.norm([diff_x, diff_y, diff_z]))
            return self._normalize_angle( expected_elevation )
        elif isinstance(meas, ElevationGlobal_Explicit) or isinstance(meas, ElevationGlobal_Implicit):
            src_x = x_hat[meas.src_id*self.num_ownship_states,0]
            src_y = x_hat[meas.src_id*self.num_ownship_states+1,0]
            src_z = x_hat[meas.src_id*self.num_ownship_states+2,0]
            other_x = meas.global_pos[0]
            other_y = meas.global_pos[1]
            other_z = meas.global_pos[2]
            diff_x = other_x - src_x
            diff_y = other_y - src_y
            diff_z = other_z - src_z

            # Protect division by zero
            diff_x = diff_x if abs(diff_x) > 0.01 else 0.01
            diff_y = diff_y if abs(diff_y) > 0.01 else 0.01
            diff_z = diff_z if abs(diff_z) > 0.01 else 0.01

            expected_elevation = np.arcsin(diff_z / np.linalg.norm([diff_x, diff_y, diff_z]))
            return self._normalize_angle( expected_elevation )
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
                src_id = range_meas[0].src_id
                src_yaw = self.x_hat[ src_id * self.num_ownship_states + 2,0]
                cov_ori = az + src_yaw
                mean_x = r * np.cos(cov_ori) + self.x_hat[src_id*self.num_ownship_states,0]
                mean_y = r * np.sin(cov_ori) + self.x_hat[src_id*self.num_ownship_states+1,0]
                prev_state = deepcopy(self.x_hat)
                # Find covariance from eigendata
                r_var = range_meas[0].R
                az_var = az_meas[0].R
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
        else: # 3D
            range_meas = [x for x in self.meas_queue if (isinstance(x, Range_Explicit) and x.measured_asset == asset_id)]
            az_meas = [x for x in self.meas_queue if (isinstance(x, Azimuth_Explicit) and x.measured_asset == asset_id)]
            el_meas = [x for x in self.meas_queue if (isinstance(x, Elevation_Explicit) and x.measured_asset == asset_id)]
            if not range_meas or not az_meas or not el_meas: # still waiting on other meas
                return
            else:
                r = range_meas[0].data
                az = az_meas[0].data
                el = el_meas[0].data
                src_id = range_meas[0].src_id
                src_yaw = self.x_hat[ src_id * self.num_ownship_states + 3,0]
                cov_ori = az + src_yaw
                r_xy = r * np.cos(el)

                mean_x = r_xy * np.cos(cov_ori) + self.x_hat[src_id*self.num_ownship_states,0]
                mean_y = r_xy * np.sin(cov_ori) + self.x_hat[src_id*self.num_ownship_states+1,0]
                mean_z = r * np.sin(el) + self.x_hat[src_id*self.num_ownship_states+2,0]

                # Find covariance from eigendata
                r_var = range_meas[0].R
                az_var = az_meas[0].R
                el_var = el_meas[0].R

                eig_az = r_xy * np.tan(az_var)
                e_vec = np.array([[np.cos(cov_ori), np.sin(cov_ori)]]).T
                e_vec2 = np.array([[np.cos(cov_ori + np.pi/2), np.sin(cov_ori + np.pi/2)]]).T
                S = np.concatenate((e_vec, e_vec2), axis=1)
                D = np.zeros((2,2)); D[0,0] = r_var; D[1,1] = eig_az # The sd isn't r_var exactly, but close enough
                A = np.dot( np.dot(S,D), np.linalg.inv(S) )
                self.x_hat[asset_id*self.num_ownship_states,0] = mean_x
                self.x_hat[asset_id*self.num_ownship_states+1,0] = mean_y
                self.x_hat[asset_id*self.num_ownship_states+2,0] = mean_z

                # Zero out cov columns
                for col in range(self.num_ownship_states):
                    self.P[:, asset_id*self.num_ownship_states+col] = np.zeros(self.num_states)
                for row in range(self.num_ownship_states):
                    self.P[asset_id*self.num_ownship_states+row,:] = np.zeros(self.num_states)
                # X by Y uncertainty
                self.P[asset_id*self.num_ownship_states:asset_id*self.num_ownship_states+2, asset_id*self.num_ownship_states:asset_id*self.num_ownship_states+2] = A
                # Z by Z uncertainty
                self.P[asset_id*self.num_ownship_states+2, asset_id*self.num_ownship_states+2] = (r * np.sin( np.sqrt(el_var) )) ** 2
                # Yaw uncertainty
                self.P[asset_id*self.num_ownship_states+3,asset_id*self.num_ownship_states+3] = 9
                self.meas_queue.remove(range_meas[0])
                self.meas_queue.remove(az_meas[0])
                self.meas_queue.remove(el_meas[0])

                # print("spawning other asset at: ")
                # print(self.x_hat[asset_id*self.num_ownship_states:(asset_id+1)*self.num_ownship_states,0])
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
            for i in range(self.num_assets):
                asset_yaw_index = i*self.num_ownship_states + 3
                state_vector[asset_yaw_index,0] = self._normalize_angle(state_vector[asset_yaw_index,0])
        return state_vector

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
        speed_index, angular_vel_index = None, None
        u_speed_index, u_ang_vel_index = None, None
        theta_index = None

        # Configure Indices of velocities so code remains valid for 2D and 3D
        if self.world_dim == 2:
            speed_index = 3
            angular_vel_index = 5
            u_speed_index = 0
            u_ang_vel_index = 1
            theta_index = 2
        else: # world dim 3
            speed_index = 4
            angular_vel_index = 7
            u_speed_index = 0
            u_ang_vel_index = 2
            theta_index = 3
            
        if self.is_main_fitler:
            self.x_hat[self.my_id * self.num_ownship_states + 6] = u[1,0] # depth speed
            self.x_hat[self.my_id * self.num_ownship_states + speed_index] = u[u_speed_index,0] # speed
            self.x_hat[self.my_id * self.num_ownship_states + angular_vel_index] = u[u_ang_vel_index,0] # angular velocity

        G = np.eye(self.num_states)
        for a in range(self.num_assets): # Loop through all assets
            start_index = a*self.num_ownship_states

            # Get this asset's control input (either actual 'u' or assume constant velocity)
            s = self.x_hat[start_index + speed_index,0]
            theta_dot = self.x_hat[start_index + angular_vel_index,0]
            theta_initial = self.x_hat[start_index+theta_index,0]

            # Runge Kutta approximation of the control inputs effect on assets position
            def dynamics(t, z):
                _x_dot = s * np.cos(z[2])
                _y_dot = s * np.sin(z[2])
                _theta_dot = theta_dot
                return np.array([_x_dot, _y_dot, _theta_dot])

            t_init, t_final = 0, 1
            z_init = np.concatenate( (self.x_hat[start_index:start_index + 2,0], np.array([theta_initial])) )
            r = integrate.RK45(dynamics, t_init, z_init, t_final)
            while r.status == "running":
                status = r.step()

            self.x_hat[start_index:start_index+2,0] = r.y[:2]
            self.x_hat[start_index+theta_index,0] = r.y[2]
            
            # Construct this asset's part of jacobian
            if self.world_dim == 2:
                G[start_index + 2, start_index + 5] = 1
                G[start_index, start_index + 2] = -s * np.sin(theta_initial + theta_dot/2)
                G[start_index + 1, start_index + 2] = s * np.cos(theta_initial + theta_dot/2)
                G[start_index, start_index + 3] = np.cos(theta_initial + theta_dot/2)
                G[start_index+1, start_index + 3] = np.sin(theta_initial + theta_dot/2)

                G[start_index, start_index + 5] = (-s * np.sin(theta_initial + theta_dot/2)) / 2
                G[start_index+1, start_index + 5] =  (s*np.cos(theta_initial + theta_dot/2)) / 2
            else: # 3D World
                G[start_index+2, start_index+6] = 1
                G[start_index + 3, start_index + 7] = 1 # theta and theta dot
                # dx/dtheta, dy/dtheta
                G[start_index, start_index + 3] = -s * np.sin(theta_initial + theta_dot/2)
                G[start_index + 1, start_index + 3] = s * np.cos(theta_initial + theta_dot/2)
                # dx/ds, dy/ds
                G[start_index, start_index + 4] = np.cos(theta_initial + theta_dot/2)
                G[start_index+1, start_index + 4] = np.sin(theta_initial + theta_dot/2)
                # dx/dtheta_dot, dy/dtheta_dot
                G[start_index, start_index + 7] = (-s * np.sin(theta_initial + theta_dot/2)) / 2
                G[start_index+1, start_index + 7] =  (s*np.cos(theta_initial + theta_dot/2)) / 2
                
                # Propagate Depth
                self.x_hat[start_index+2,0] = self.x_hat[start_index+2,0] + self.x_hat[start_index+6,0]
            
        return G

""" Main filter
differs slightly from an ETFilter in its implicit measurement update
Needs access to common filters for implicit measurement updates
"""
class ETFilter_Main( ETFilter ):
    def __init__(self, my_id, num_ownship_states, world_dim, x0, P0, linear_dynamics, common_filters):
        """
        common_filters : dict
            key : int
                other asset id
            value : ETFiler
                common filter between both assets
        """
        super(ETFilter_Main, self).__init__(my_id, num_ownship_states, world_dim, x0, P0, linear_dynamics)
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
        if meas.is_angle_meas:
            mu = self._normalize_angle(mu)
            alpha = self._normalize_angle(alpha)
        return mu, Qe, alpha

    def _get_common_filter_states(self, asset_id):
        return self.common_filters[asset_id]