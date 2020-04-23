from __future__ import division
from etddf.measurement import *

# TODO instantinate on FromGlobalRange/Azimuth!!
# def instantiate_asset_range_bearing(self, asset_id):
#     if self.world_dim == 2:
#         range_meas = [x for x in self.meas_queue if (isinstance(x, Range_Explicit) and x.measured_asset_id == asset_id)]
#         az_meas = [x for x in self.meas_queue if (isinstance(x, Azimuth_Explicit) and x.measured_asset_id == asset_id)]
#         if not range_meas or not az_meas: # still waiting on other meas
#             return
#         else: # instantiate gaussian of asset at measurement mean
#             r = range_meas[0].data
#             az = az_meas[0].data
#             src_id = range_meas[0].src_id
#             src_yaw = self.x_hat[ src_id * self.num_ownship_states + 2,0]
#             cov_ori = az + src_yaw
#             mean_x = r * np.cos(cov_ori) + self.x_hat[src_id*self.num_ownship_states,0]
#             mean_y = r * np.sin(cov_ori) + self.x_hat[src_id*self.num_ownship_states+1,0]
#             prev_state = deepcopy(self.x_hat)
#             # Find covariance from eigendata
#             r_var = range_meas[0].R
#             az_var = az_meas[0].R
#             eig_az = r * np.tan(az_var)
#             e_vec = np.array([[np.cos(cov_ori), np.sin(cov_ori)]]).T
#             e_vec2 = np.array([[np.cos(cov_ori + np.pi/2), np.sin(cov_ori + np.pi/2)]]).T
#             S = np.concatenate((e_vec, e_vec2), axis=1)
#             D = np.zeros((2,2)); D[0,0] = r_var; D[1,1] = eig_az
#             A = np.dot( np.dot(S,D), np.linalg.inv(S) )
#             self.x_hat[asset_id*self.num_ownship_states,0] = mean_x
#             self.x_hat[asset_id*self.num_ownship_states+1,0] = mean_y

#             # Zero out cov columns
#             for col in range(self.num_ownship_states):
#                 self.P[:, asset_id*self.num_ownship_states+col] = np.zeros(self.num_states)
#             for row in range(self.num_ownship_states):
#                 self.P[asset_id*self.num_ownship_states+row,:] = np.zeros(self.num_states)
#             self.P[asset_id*self.num_ownship_states:asset_id*self.num_ownship_states+2, asset_id*self.num_ownship_states:asset_id*self.num_ownship_states+2] = A
#             self.P[asset_id*self.num_ownship_states+2,asset_id*self.num_ownship_states+2] = 9
#             self.meas_queue.remove(range_meas[0])
#             self.meas_queue.remove(az_meas[0])
#     else: # 3D
#         range_meas = [x for x in self.meas_queue if (isinstance(x, Range_Explicit) and x.measured_asset_id == asset_id)]
#         az_meas = [x for x in self.meas_queue if (isinstance(x, Azimuth_Explicit) and x.measured_asset_id == asset_id)]
#         el_meas = [x for x in self.meas_queue if (isinstance(x, Elevation_Explicit) and x.measured_asset_id == asset_id)]
#         if not range_meas or not az_meas or not el_meas: # still waiting on other meas
#             return
#         else:
#             r = range_meas[0].data
#             az = az_meas[0].data
#             el = el_meas[0].data
#             src_id = range_meas[0].src_id
#             src_yaw = self.x_hat[ src_id * self.num_ownship_states + 3,0]
#             cov_ori = az + src_yaw
#             r_xy = r * np.cos(el)

#             mean_x = r_xy * np.cos(cov_ori) + self.x_hat[src_id*self.num_ownship_states,0]
#             mean_y = r_xy * np.sin(cov_ori) + self.x_hat[src_id*self.num_ownship_states+1,0]
#             mean_z = r * np.sin(el) + self.x_hat[src_id*self.num_ownship_states+2,0]

#             # Find covariance from eigendata
#             r_var = range_meas[0].R
#             az_var = az_meas[0].R
#             el_var = el_meas[0].R

#             eig_az = r_xy * np.tan(az_var)
#             e_vec = np.array([[np.cos(cov_ori), np.sin(cov_ori)]]).T
#             e_vec2 = np.array([[np.cos(cov_ori + np.pi/2), np.sin(cov_ori + np.pi/2)]]).T
#             S = np.concatenate((e_vec, e_vec2), axis=1)
#             D = np.zeros((2,2)); D[0,0] = r_var; D[1,1] = eig_az # The sd isn't r_var exactly, but close enough
#             A = np.dot( np.dot(S,D), np.linalg.inv(S) )
#             self.x_hat[asset_id*self.num_ownship_states,0] = mean_x
#             self.x_hat[asset_id*self.num_ownship_states+1,0] = mean_y
#             self.x_hat[asset_id*self.num_ownship_states+2,0] = mean_z

#             # Zero out cov columns
#             for col in range(self.num_ownship_states):
#                 self.P[:, asset_id*self.num_ownship_states+col] = np.zeros(self.num_states)
#             for row in range(self.num_ownship_states):
#                 self.P[asset_id*self.num_ownship_states+row,:] = np.zeros(self.num_states)
#             # X by Y uncertainty
#             self.P[asset_id*self.num_ownship_states:asset_id*self.num_ownship_states+2, asset_id*self.num_ownship_states:asset_id*self.num_ownship_states+2] = A
#             # Z by Z uncertainty
#             self.P[asset_id*self.num_ownship_states+2, asset_id*self.num_ownship_states+2] = (r * np.sin( np.sqrt(el_var) )) ** 2
#             # Yaw uncertainty
#             self.P[asset_id*self.num_ownship_states+3,asset_id*self.num_ownship_states+3] = 9
#             self.meas_queue.remove(range_meas[0])
#             self.meas_queue.remove(az_meas[0])
#             self.meas_queue.remove(el_meas[0])

#             # print("spawning other asset at: ")
#             # print(self.x_hat[asset_id*self.num_ownship_states:(asset_id+1)*self.num_ownship_states,0])
#             # raise Exception("ya done")