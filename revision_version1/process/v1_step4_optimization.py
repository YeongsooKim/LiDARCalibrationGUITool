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
from tqdm import tqdm

# User defined modules
from process import utils_icp
from process import utils_pointcloud
from process import utils_cost_func

class Optimization:
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
        start_time = args[0]
        end_time = args[1]
        PARM_LIDAR = args[2]
        df_info = self.importing.df_info

        # Limit time
        df_info = df_info.drop(
            df_info[(df_info.index < start_time) | (df_info.index > end_time)].index)
        ##############################################################################################################################
        # %% 3. Multiple optimization
        ##############################################################################################################################

        # -----------------------------------------------------------------------------------------------------------------------------
        # 3-1.  Accumulation
        # -----------------------------------------------------------------------------------------------------------------------------

        ##################
        # Get calibration data
        idxSensor = PARM_LIDAR['PrincipalSensor']
        calib_param = self.CalibrationParam[idxSensor]

        ##################
        # Remove rows by other sensors
        strColIndex = 'PointCloud_' + str(idxSensor)
        df_one_info = df_info[['east_m', 'north_m', 'heading', strColIndex]]
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
            if idxSensor == PARM_LIDAR['PrincipalSensor']:
                thread.emit_string.emit(str('Complete LiDAR {} calibration'.format(idxSensor)))
                thread.mutex.unlock()
                continue

            # Get calibration data
            calib_param = self.CalibrationParam[idxSensor]

            # Remove rows by other sensors
            strColIndex = 'PointCloud_' + str(idxSensor)
            df_one_info = df_info[['east_m', 'north_m', 'heading', strColIndex]]
            df_one_info = df_one_info.drop(df_info[(df_one_info[strColIndex].values == 0)].index)

            ##### Arguments
            # Get position
            pose = df_one_info['east_m'].values
            pose = np.vstack([pose, df_one_info['north_m'].values])
            pose = np.vstack([pose, df_one_info['heading'].values * np.pi / 180.])
            pose = np.vstack([pose, df_one_info[strColIndex].values])

            # Get Point cloud list
            pointcloud = self.importing.PointCloudSensorList[idxSensor]

            ##### cost function for optimization
            utils_cost_func.strFile = 'PointCloud_' + str(idxSensor)
            utils_cost_func.nFeval = 0
            res = minimize(utils_cost_func.compute_multi_err,
                           calib_param[2],
                           args=(calib_param[3], calib_param[4], pose, pointcloud, accum_point_enup, nearest_neighbor,
                                 self.config.PARM_IM, self.config.PARM_MO, thread),
                           thread=thread,
                           method='Powell',
                           options={'ftol': 1e-10, 'disp': True})
            thread.emit_string.emit(str('Complete LiDAR {} calibration'.format(idxSensor)))
            thread.mutex.unlock()
            # set data
            self.CalibrationParam[idxSensor][2] = float(res.x)

        self.calib_yaw.clear()
        self.calib_x.clear()
        self.calib_y.clear()
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            self.calib_yaw.append(self.CalibrationParam[idxSensor][2] * 180 / 3.141592)
            self.calib_x.append(self.CalibrationParam[idxSensor][3])
            self.calib_y.append(self.CalibrationParam[idxSensor][4])

        print("Complete optimization calibration")
