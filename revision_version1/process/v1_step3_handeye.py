# -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
@date: 2018-07-12
@version: 0.0.1
"""

##############################################################################################################################
# %% Import libraries
##############################################################################################################################

# Basic modules in Anaconda
import numpy as np
import matplotlib.pyplot as plt

import configparser

# Additional modules
from tqdm import tqdm

# User defined modules
from process import utils_icp
from process import utils_file
from process import utils_pointcloud
from process import utils_pose

class HandEye:
    def __init__(self, config, Import):
        self.config = config
        self.Import = Import
        self.complete_calibration = False

        # Path and file
        self.export_path = ''
        self.result_calibration_config_file = ''

        # Parameter
        self.progress = 0.0

        self.CalibrationParam = {}

    ##############################################################################################################################
    # %% 3. Handeye
    ##############################################################################################################################
    def Calibration(self, thread):
        thread.mutex.lock()
        start_time = self.Import.start_time
        end_time = self.Import.end_time
        self.df_info = self.Import.df_info

        # Limit time
        self.df_info = self.df_info.drop(
            self.df_info[(self.df_info.index < start_time) | (self.df_info.index > end_time)].index)
        # -----------------------------------------------------------------------------------------------------------------------------
        # 3-1. Match the point cloud based on ICP
        # -----------------------------------------------------------------------------------------------------------------------------
        diff_point_xyzdh_dict = {}
        diff_gnss_xyzdh_dict = {}
        lidar_len = len(self.config.PARM_LIDAR['SensorList'])
        p_index = 0.0
        for idxSensor in self.config.PARM_LIDAR['SensorList']:
            diff_point_xyzdh = []
            diff_gnss_xyzdh = []

            # Remove rows by other sensors
            strColIndex = 'PointCloud_' + str(idxSensor)
            df_one_info = self.df_info[['east_m', 'north_m', 'heading', strColIndex]]
            df_one_info = df_one_info.drop(self.df_info[(df_one_info[strColIndex].values == 0)].index)

            # Sampling based on interval
            df_sampled_info = df_one_info.iloc[::self.config.PARM_HE['SamplingInterval'], :]

            # Generate Index Pairs
            idx_pair = []
            l = list(range(len(df_sampled_info)))
            for first, second in zip(l, l[1:]):
                idx_pair.append((first, second))

            # Compute ICP
            pbar = tqdm(idx_pair)
            iteration_size = len(idx_pair)
            self.progress = 0.0
            index = 0
            for i, j in pbar:
                # Display progress
                iteration_ratio = (float(index + 1) / float(iteration_size)) / lidar_len
                percentage = (iteration_ratio + p_index / float(lidar_len)) * 100
                if percentage >= 99.5:
                    percentage = 100.0
                thread.change_value.emit(int(percentage))

                pbar.set_description("PointCloud_" + str(idxSensor))
                # Get point clouds
                pointcloud1 = self.Import.PointCloudSensorList[idxSensor][int(df_sampled_info[strColIndex].values[i])]
                pointcloud2 = self.Import.PointCloudSensorList[idxSensor][int(df_sampled_info[strColIndex].values[j])]

                if pointcloud1.shape[0] < 1:
                    continue
                if pointcloud2.shape[0] < 1:
                    continue

                # Get GNSS
                e1 = df_sampled_info['east_m'].values[i]
                n1 = df_sampled_info['north_m'].values[i]
                h1 = df_sampled_info['heading'].values[i]

                e2 = df_sampled_info['east_m'].values[j]
                n2 = df_sampled_info['north_m'].values[j]
                h2 = df_sampled_info['heading'].values[j]

                diff_e = e2 - e1
                diff_n = n2 - n1
                diff_h = (h2 - h1) * np.pi / 180.

                # Translation in vehicle coordinate based on GNSS (rely on the initial heading)
                h1_rad = (h1 + 90.) * np.pi / 180.
                translation_gnss = np.matmul(np.array(([np.cos(-h1_rad), -np.sin(-h1_rad), 0., 0.],
                                                       [np.sin(-h1_rad), np.cos(-h1_rad), 0., 0.],
                                                       [0., 0., 1., 0.],
                                                       [0., 0., 0., 1.])),
                                             np.array([diff_e, diff_n, 0., 1.]))

                # Generate Transform
                transform_gnss = np.array(([np.cos(diff_h), -np.sin(diff_h), 0., translation_gnss[0]],
                                           [np.sin(diff_h), np.cos(diff_h), 0., translation_gnss[1]],
                                           [0., 0., 1., 0.],
                                           [0., 0., 0., 1.]))
                # ICP
                transform_point, distances, iterations, converged = utils_icp.icp_NM(pointcloud2[:, 0:3],
                                                                                     pointcloud1[:, 0:3],
                                                                                     init_pose=transform_gnss,
                                                                                     tolerance=self.config.PARM_HE['Tolerance'],
                                                                                     max_iterations=self.config.PARM_HE[
                                                                                         'MaximumIteration'],
                                                                                     rm_outlier_dist=self.config.PARM_HE[
                                                                                         'OutlierDistance_m'])

                #        # Check Matching of ICP
                #        uf.FnDisplayTransformPointCloud(pointcloud2, pointcloud1, transform_point)

                if converged == False:
                    continue

                # Get difference of x,y,z,d,h
                diff_point = np.append(transform_point[0:3, 3],
                                       np.sqrt(np.power(transform_point[0, 3], 2) + np.power(transform_point[1, 3], 2)))
                diff_point = np.append(diff_point, np.arctan2(transform_point[1, 0], transform_point[0, 0]) * 180. / np.pi)
                diff_gnss = np.append(transform_gnss[0:3, 3],
                                      np.sqrt(np.power(transform_gnss[0, 3], 2) + np.power(transform_gnss[1, 3], 2)))
                diff_gnss = np.append(diff_gnss, np.arctan2(transform_gnss[1, 0], transform_gnss[0, 0]) * 180. / np.pi)

                # Get errors between gnss and point
                error_heading = np.abs(diff_gnss[4] - diff_point[4])
                error_distance = np.abs(diff_gnss[3] - diff_point[3])

                # Check validation using parameter
                if (float(error_heading) < self.config.PARM_HE['filter_HeadingThreshold']) & (
                        float(error_distance) < self.config.PARM_HE['filter_DistanceThreshold']):
                    diff_point_xyzdh.append(np.transpose(diff_point).tolist())
                    diff_gnss_xyzdh.append(np.transpose(diff_gnss).tolist())

                index = index + 1

            diff_point_xyzdh_dict[idxSensor] = diff_point_xyzdh
            diff_gnss_xyzdh_dict[idxSensor] = diff_gnss_xyzdh

            p_index = p_index + 1.0

        thread.mutex.unlock()

        # -----------------------------------------------------------------------------------------------------------------------------
        # 3-2. Solve the A*X=X*B
        # -----------------------------------------------------------------------------------------------------------------------------

        # Construction of matrix for hand eye calibration
        self.label = []
        self.calib_x = []
        self.calib_y = []
        self.calib_yaw = []
        for idxSensor in self.config.PARM_LIDAR['SensorList']:
            section_name = 'LiDAR_' + str(idxSensor)

            diff_point_xyzdh = diff_point_xyzdh_dict[idxSensor]
            diff_gnss_xyzdh = diff_gnss_xyzdh_dict[idxSensor]

            Augment_RT = np.empty((0, 4))
            Augment_t_veh = np.array([])
            for i in list(range(len(diff_point_xyzdh))):
                # Rotation and translation matrixes for accumulated values
                diff_gnss_heading = diff_gnss_xyzdh[i][4]
                diff_gnss_heading_rad = diff_gnss_heading * np.pi / 180.
                R_veh = np.array([
                    [np.cos(diff_gnss_heading_rad), -np.sin(diff_gnss_heading_rad)],
                    [np.sin(diff_gnss_heading_rad), np.cos(diff_gnss_heading_rad)]])
                diff_point_heading = diff_point_xyzdh[i][4]
                diff_point_heading_rad = diff_point_heading * np.pi / 180.
                R_lidar = np.array([
                    [np.cos(diff_point_heading_rad), -np.sin(diff_point_heading_rad)],
                    [np.sin(diff_point_heading_rad), np.cos(diff_point_heading_rad)]])
                t_veh = np.transpose(np.array(diff_gnss_xyzdh[i][0:2]))
                t_lidar = np.transpose(np.array(diff_point_xyzdh[i][0:2]))
                modified_t_lidar = np.array([[-t_lidar[0], t_lidar[1]], [-t_lidar[1], -t_lidar[0]]])

                # A*X = B where X is calibration parameter matrix [tx; ty; cos(heading); sin(heading)]
                tmp = np.array(R_veh - np.eye(2))
                tmp = np.hstack([tmp, modified_t_lidar])

                # add Augment_RT and Augment_t_veh
                Augment_RT = np.vstack([Augment_RT, tmp])
                Augment_t_veh = np.hstack([Augment_t_veh, -t_veh])

            # Solve the A*X = B*X
            X = np.matmul(np.linalg.pinv(Augment_RT), Augment_t_veh)
            Rot_sen2veh = np.array([[X[2], -X[3]], [X[3], X[2]]])
            Trans_veh2sen = np.array([X[0], X[1]])
            yaw = np.arctan2(Rot_sen2veh[1, 0], Rot_sen2veh[0, 0])

            calib = []
            calib.append(0.)  # roll
            calib.append(0.)  # pitch
            calib.append(yaw)  # yaw
            calib.append(Trans_veh2sen[0])  # x
            calib.append(Trans_veh2sen[1])  # y
            calib.append(0.)  # z
            self.CalibrationParam[idxSensor] = calib
            self.label.append(section_name)
            self.calib_yaw.append(yaw * 180 / 3.141592)
            self.calib_x.append(Trans_veh2sen[0])
            self.calib_y.append(Trans_veh2sen[1])

        self.complete_calibration = True
        print("Complete calibration")