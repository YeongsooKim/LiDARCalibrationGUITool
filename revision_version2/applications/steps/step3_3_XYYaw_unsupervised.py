# -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
@date: 2018-07-12
@version: 0.0.1
"""

# Basic modules in Anaconda
import numpy as np
from sklearn.neighbors import NearestNeighbors
from scipy.optimize import minimize

# Additional modules

# User defined modules
from applications.utils import utils_pointcloud, utils_cost_func
import copy

class Unsupervised:
    def __init__(self, config, importing):
        self.config = config
        self.importing = importing

        # Path and file
        self.config_lidar_file = ''
        self.export_path = ''
        self.result_calibration_config_file = ''

        # Parameter
        self.progress = ''
        self.complete_calibration = False

        self.CalibrationParam = {}
        self.calib_yaw = []
        self.calib_x = []
        self.calib_y = []

    ##############################################################################################################################
    # %% 3. Optimization
    ##############################################################################################################################
    def Calibration(self, thread, args):
        thread.emit_string.emit(str('Start optimization calibration'))
        # thread.emit_string.emit(str('LiDAR          Iter  Opti_value   Cost')){0:>10}   {1:4d} {2:>13} {3:>13}
        thread.emit_string.emit(str('     {0:>10}    {1:>4}  {2:>15}      {3:>15}'.format('LiDAR', 'Iter', 'Opti_Value', 'Cost')))
        mutex_unlock = False
        start_time = args[0]
        end_time = args[1]
        PARM_LIDAR = copy.deepcopy(args[2])
        using_gnss_motion = args[3]
        vehicle_speed_threshold = args[4] / 3.6
        min_thresh_z_m = args[5]
        print('unsupervised min_thresh_z_m {}'.format(min_thresh_z_m))
        zrp_calib = args[6]
        is_single_optimization = args[7]
        # print(zrp_calib)
        df_info = copy.deepcopy(self.importing.df_info)
        # Limit time
        df_info = df_info.drop(
            df_info[(df_info.index < start_time) | (df_info.index > end_time)].index)

        if using_gnss_motion:
            df_info = df_info.drop(df_info[df_info['speed_x'] < vehicle_speed_threshold].index)

        ##############################################################################################################################
        # %% 3. Multiple optimization
        ##############################################################################################################################

        # -----------------------------------------------------------------------------------------------------------------------------
        # 3-1.  Accumulation
        # -----------------------------------------------------------------------------------------------------------------------------

        ##################
        # Get calibration data
        if is_single_optimization:
            idxSensor = PARM_LIDAR['SingleSensor']
        else:
            idxSensor = PARM_LIDAR['PrincipalSensor']
        calib_param = self.CalibrationParam[idxSensor]

        ##################
        # Remove rows by other sensors
        strColIndex = 'XYZRGB_' + str(idxSensor)

        ## Calibration Initialization            
        dZ_m = zrp_calib[idxSensor][0]
        dRoll_rad = zrp_calib[idxSensor][1] * np.pi / 180.0
        dPitch_rad = zrp_calib[idxSensor][2] * np.pi / 180.0
        
        # print(dZ_m)
        
        tf_RollPitchCalib = np.array([[np.cos(dPitch_rad), np.sin(dPitch_rad)*np.sin(dRoll_rad), np.sin(dPitch_rad)*np.cos(dRoll_rad), 0.],
                              [0., np.cos(dRoll_rad), -1*np.sin(dRoll_rad), 0.],
                              [-1*np.sin(dPitch_rad), np.cos(dPitch_rad)*np.sin(dRoll_rad), np.cos(dPitch_rad)*np.cos(dRoll_rad), dZ_m],
                              [0., 0., 0., 1.]])   
        
        if not using_gnss_motion:
            df_one_info = df_info[['east_m', 'north_m', 'heading', strColIndex]]
        elif using_gnss_motion:
            df_one_info = df_info[['dr_east_m', 'dr_north_m', 'dr_heading', strColIndex]]
            df_one_info.rename(columns={"dr_east_m": "east_m", "dr_north_m": "north_m", "dr_heading": "heading"},
                               inplace=True)
        df_one_info = df_one_info.drop(df_info[(df_one_info[strColIndex].values == 0)].index)

        ##################
        ##### Arguments
        # Get position
        pose = df_one_info['east_m'].values
        pose = np.vstack([pose, df_one_info['north_m'].values])
        pose = np.vstack([pose, df_one_info['heading'].values * np.pi / 180.])
        pose = np.vstack([pose, df_one_info[strColIndex].values])
        index_pointcloud = pose[3]

        ##################
        # Get Point cloud list
        pointcloud = self.importing.PointCloudSensorList[idxSensor]
        # print("\n############\n")
        # print(len(pointcloud))
        # print(pointcloud)
        tmp_pc = []
        for key in pointcloud:
            pointcloud_lidar = pointcloud[key]
            #
            # pointcloud_lidar_homogeneous = np.insert(pointcloud_lidar, 3, 1, axis = 1)
            #
            # # PointCloud Conversion: Roll, Pitch, Height
            # pointcloud_in_lidar_frame_calibrated_rollpitch = np.matmul(tf_RollPitchCalib, np.transpose(pointcloud_lidar_homogeneous))
            # pointcloud_in_lidar_frame_calibrated_rollpitch = np.delete(pointcloud_in_lidar_frame_calibrated_rollpitch, 3, axis = 0)
            # pointcloud_in_lidar_frame_calibrated_rollpitch = np.transpose(pointcloud_in_lidar_frame_calibrated_rollpitch)

            remove_filter = pointcloud_lidar[:, 2] < float(min_thresh_z_m)

            filtered_pointcloud_lidar = np.ma.compress(np.bitwise_not(np.ravel(remove_filter)),
                                                        pointcloud_lidar[:, 0:3], axis=0)
            filtered_pointcloud_lidar = np.array(filtered_pointcloud_lidar)

            pointcloud_lidar_homogeneous = np.insert(filtered_pointcloud_lidar, 3, 1, axis = 1)

            pointcloud_in_lidar_frame_calibrated_rollpitch = np.matmul(tf_RollPitchCalib, np.transpose(pointcloud_lidar_homogeneous))
            pointcloud_in_lidar_frame_calibrated_rollpitch = np.delete(pointcloud_in_lidar_frame_calibrated_rollpitch, 3, axis = 0)
            pointcloud_in_lidar_frame_calibrated_rollpitch = np.transpose(pointcloud_in_lidar_frame_calibrated_rollpitch)

            tmp_pc.append(pointcloud_in_lidar_frame_calibrated_rollpitch)

        pointcloud = tmp_pc
        # print("\n############\n")
        # print(pointcloud)

        if is_single_optimization:
            thread.mutex.lock()

            ##### cost function for optimization
            utils_cost_func.strFile = 'XYZRGB_' + str(idxSensor)
            utils_cost_func.nFeval = 0
            res = minimize(utils_cost_func.compute_single_err,
                           calib_param[2],
                           args=(calib_param[3], calib_param[4], pose, pointcloud,
                                 self.config.PARM_IM, self.config.PARM_SO, thread),
                           thread=thread,
                           method='Powell',
                           options={'ftol': 1e-10, 'disp': True})
            # set data
            self.CalibrationParam[idxSensor][2] = float(res.x)

            if not thread._status:
                thread.emit_string.emit(str('Stop LiDAR {} calibration'.format(idxSensor)))
            thread.emit_string.emit(str('Complete LiDAR {} calibration'.format(idxSensor)))

            self.calib_yaw.clear()
            self.calib_x.clear()
            self.calib_y.clear()

            self.calib_yaw.append(self.CalibrationParam[idxSensor][2] * 180 / 3.141592)
            self.calib_x.append(self.CalibrationParam[idxSensor][3])
            self.calib_y.append(self.CalibrationParam[idxSensor][4])

            self.PARM_LIDAR = copy.deepcopy(PARM_LIDAR)
            thread.mutex.unlock()
            print("Complete single-optimization calibration")
        else:
            ##################
            # Sampling the pose bsaed on pose sampling interval
            num_pose = pose.shape[1]
            interval = self.config.PARM_IM['SamplingInterval']
            if interval < 1:
                interval = 1
            idx_sampling_pose = list(range(0, num_pose, interval))

            ##################
            # Accumulation of point cloud
            accum_point_enup = np.empty((0, 4))
            for idx_pose in idx_sampling_pose:
                # Convert raw to enu
                point_sensor = pointcloud[int(index_pointcloud[idx_pose])][:, 0:3]
                point_enu = utils_pointcloud.cvt_pointcloud_6dof_sensor_enu(point_sensor, calib_param, pose[0:3, idx_pose])

                # Add index
                point_enup = np.concatenate((point_enu, np.full((point_enu.shape[0], 1), idx_pose)), axis=1)

                # Accumulate the point cloud
                accum_point_enup = np.vstack([accum_point_enup, point_enup])

            ##################
            # Generate nearest neighbors
            nearest_neighbor = NearestNeighbors(n_neighbors=self.config.PARM_MO['NumPointsPlaneModeling'])
            nearest_neighbor.fit(accum_point_enup)

            # -----------------------------------------------------------------------------------------------------------------------------
            # 3-2.  Optimization
            # -----------------------------------------------------------------------------------------------------------------------------

            for idxSensor in PARM_LIDAR['CheckedSensorList']:
                # Exclude the principal sensor
                thread.mutex.lock()
                mutex_unlock = False
                if idxSensor == PARM_LIDAR['PrincipalSensor']:
                    thread.emit_string.emit(str('Complete LiDAR {} calibration'.format(idxSensor)))
                    thread.mutex.unlock()
                    mutex_unlock = True
                    continue

                # Get calibration data
                calib_param = self.CalibrationParam[idxSensor]

                # Remove rows by other sensors
                strColIndex = 'XYZRGB_' + str(idxSensor)

                ## Calibration Initialization            
                dZ_m = zrp_calib[idxSensor][0]
                dRoll_rad = zrp_calib[idxSensor][1] * np.pi / 180.0
                dPitch_rad = zrp_calib[idxSensor][2] * np.pi / 180.0
                
                # print(dZ_m)
                
                
                tf_RollPitchCalib = np.array([[np.cos(dPitch_rad), np.sin(dPitch_rad)*np.sin(dRoll_rad), np.sin(dPitch_rad)*np.cos(dRoll_rad), 0.],
                                      [0., np.cos(dRoll_rad), -1*np.sin(dRoll_rad), 0.],
                                      [-1*np.sin(dPitch_rad), np.cos(dPitch_rad)*np.sin(dRoll_rad), np.cos(dPitch_rad)*np.cos(dRoll_rad), dZ_m],
                                      [0., 0., 0., 1.]])   

                if not using_gnss_motion:
                    df_one_info = df_info[['east_m', 'north_m', 'heading', strColIndex]]
                    df_one_info = df_one_info.drop(df_info[(df_one_info[strColIndex].values == 0)].index)
                elif using_gnss_motion:
                    df_one_info = df_info[['dr_east_m', 'dr_north_m', 'dr_heading', strColIndex]]
                    df_one_info.rename(columns={"dr_east_m" : "east_m", "dr_north_m" : "north_m", "dr_heading" : "heading"}, inplace=True)
                    df_one_info = df_one_info.drop(df_info[(df_one_info[strColIndex].values == 0)].index)

                ##### Arguments
                # Get position
                pose = df_one_info['east_m'].values
                pose = np.vstack([pose, df_one_info['north_m'].values])
                pose = np.vstack([pose, df_one_info['heading'].values * np.pi / 180.])
                pose = np.vstack([pose, df_one_info[strColIndex].values])

                # Get Point cloud list
                pointcloud = self.importing.PointCloudSensorList[idxSensor]

                # print("\n############\n")
                # print(pointcloud)
                tmp_pc = []
                for key in pointcloud:
                    pointcloud_lidar = pointcloud[key]

                    # pointcloud_lidar_homogeneous = np.insert(pointcloud_lidar, 3, 1, axis = 1)
                    #
                    # # PointCloud Conversion: Roll, Pitch, Height
                    # pointcloud_in_lidar_frame_calibrated_rollpitch = np.matmul(tf_RollPitchCalib, np.transpose(pointcloud_lidar_homogeneous))
                    # pointcloud_in_lidar_frame_calibrated_rollpitch = np.delete(pointcloud_in_lidar_frame_calibrated_rollpitch, 3, axis = 0)
                    # pointcloud_in_lidar_frame_calibrated_rollpitch = np.transpose(pointcloud_in_lidar_frame_calibrated_rollpitch)

                    remove_filter = pointcloud_lidar[:, 2] < float(min_thresh_z_m)

                    filtered_pointcloud_lidar = np.ma.compress(np.bitwise_not(np.ravel(remove_filter)),
                                                               pointcloud_lidar[:, 0:3], axis=0)
                    filtered_pointcloud_lidar = np.array(filtered_pointcloud_lidar)

                    pointcloud_lidar_homogeneous = np.insert(filtered_pointcloud_lidar, 3, 1, axis=1)

                    pointcloud_in_lidar_frame_calibrated_rollpitch = np.matmul(tf_RollPitchCalib, np.transpose(
                        pointcloud_lidar_homogeneous))
                    pointcloud_in_lidar_frame_calibrated_rollpitch = np.delete(
                        pointcloud_in_lidar_frame_calibrated_rollpitch, 3, axis=0)
                    pointcloud_in_lidar_frame_calibrated_rollpitch = np.transpose(
                        pointcloud_in_lidar_frame_calibrated_rollpitch)

                    tmp_pc.append(pointcloud_in_lidar_frame_calibrated_rollpitch)
                pointcloud = tmp_pc

                ##### cost function for optimization
                utils_cost_func.strFile = 'XYZRGB_' + str(idxSensor)
                utils_cost_func.nFeval = 0
                res = minimize(utils_cost_func.compute_multi_err,
                               calib_param[2],
                               args=(calib_param[3], calib_param[4], pose, pointcloud, accum_point_enup, nearest_neighbor,
                                     self.config.PARM_IM, self.config.PARM_MO, thread),
                               thread=thread,
                               method='Powell',
                               options={'ftol': 1e-10, 'disp': True})
                # set data
                self.CalibrationParam[idxSensor][2] = float(res.x)

                if not thread._status:
                    thread.emit_string.emit(str('Stop LiDAR {} calibration'.format(idxSensor)))
                    break
                thread.emit_string.emit(str('Complete LiDAR {} calibration'.format(idxSensor)))

                thread.mutex.unlock()
                mutex_unlock = True

            self.calib_yaw.clear()
            self.calib_x.clear()
            self.calib_y.clear()
            for idxSensor in PARM_LIDAR['CheckedSensorList']:
                self.calib_yaw.append(self.CalibrationParam[idxSensor][2] * 180 / 3.141592)
                self.calib_x.append(self.CalibrationParam[idxSensor][3])
                self.calib_y.append(self.CalibrationParam[idxSensor][4])

            self.PARM_LIDAR = copy.deepcopy(PARM_LIDAR)
            if not mutex_unlock:
                thread.mutex.unlock()
                mutex_unlock = True

            print("Complete mutli-optimization calibration")
