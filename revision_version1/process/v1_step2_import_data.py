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
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import open3d as o3d

import pickle
import random
from sklearn.neighbors import NearestNeighbors
from scipy.optimize import minimize
import configparser

# Additional modules
from tqdm import tqdm

# User defined modules
from process import utils_icp
from process import utils_file
from process import utils_pointcloud
from process import utils_pose


class Import:
    def __init__(self, config):
        self.config = config

        # Path and file
        self.gnss_logging_file = ''
        self.point_cloud_logging_path = ''

        # Parameter
        self.start_time = 0.0
        self.end_time = 0.0
        self.DefaultStartTime = 0.0
        self.DefaultEndTime = 0.0

        # Progress display
        self.progress = 0.0

    ##############################################################################################################################
    # %% 2. Import Logging Data
    ##############################################################################################################################
    def ParseGnss(self):
        # -----------------------------------------------------------------------------------------------------------------------------
        # 2-1. Import data
        # -----------------------------------------------------------------------------------------------------------------------------
        # Select files
        # Parse GNSS
        gnss_usecols = ['timestamp', 'latitude', 'longitude', 'heading']
        gnss_logging_file = self.gnss_logging_file + '/Gnss.csv'
        df_gnss = utils_file.parse_gnss_csv_df(gnss_logging_file, gnss_usecols)

        # Add ENU coordinate
        RefWGS84 = [df_gnss['latitude'][0], df_gnss['longitude'][0]]
        df_gnss = utils_pose.get_gnss_enu(df_gnss, RefWGS84)
        df_gnss = df_gnss.set_index('timestamp')
        self.df_info = df_gnss

        print('Parse Gnss logging data')

    def ParsePointCloud(self, thread):
        thread.mutex.lock()
        # Parse Point Cloud
        PointCloudFileCnt = 0
        self.PointCloudSensorList = {}

        self.exist_arr = []
        for idxSensor in self.config.PARM_LIDAR['SensorList']:
            if os.path.isfile(self.point_cloud_logging_path + '/PointCloud_' + str(idxSensor) + '.bin') == True:
                pointcloud_file = self.point_cloud_logging_path + '/PointCloud_' + str(idxSensor) + '.bin'
                self.exist_arr.append(1)
            else:
                self.exist_arr.append(0)

        non_error = True
        for i in self.exist_arr:
            non_error = non_error * i

        try:
            if not non_error:
                raise Exception()
        except:
            print('Raise except')

        if non_error:
            lidar_len = len(self.config.PARM_LIDAR['SensorList'])
            p_index = 0.0
            for idxSensor in self.config.PARM_LIDAR['SensorList']:
                pointcloud_file = self.point_cloud_logging_path + '/PointCloud_' + str(idxSensor) + '.bin'
                df_pointcloud = utils_file.parse_pointcloud_bin_df(pointcloud_file)
                # Set point cloud data
                pointcloud_timestamp = []
                pointcloud_index = []
                PointCloudList = {}


                pbar = tqdm(range(np.size(df_pointcloud['num_points'].values)))
                iteration_size = len(range(np.size(df_pointcloud['num_points'].values)))
                self.progress = 0.0

                index = 0
                for i in pbar:
                    # Display progress
                    iteration_ratio = (float(index + 1) / float(iteration_size))/lidar_len
                    percentage = (iteration_ratio + p_index/float(lidar_len))*100
                    if percentage >= 99.5:
                        percentage = 100.0
                    thread.change_value.emit(int(percentage))

                    # pbar.set_description('PointCloud_' + str(idxSensor) + '.bin')

                    # Get point cloud
                    arrPoint = utils_file.get_point_cloud(pointcloud_file, df_pointcloud['num_points'].values[i],
                                                          df_pointcloud['file_pointer'].values[i])
                    tmpXYZ = np.sqrt(
                        np.power(arrPoint[:, 0:1], 2) + np.power(arrPoint[:, 1:2], 2) + np.power(arrPoint[:, 2:3], 2))

                    # Check parameter
                    remove_filter = tmpXYZ < float(self.config.PARM_PC['MinThresholdDist_m'])
                    remove_filter = np.logical_or(remove_filter, tmpXYZ > float(self.config.PARM_PC['MaxThresholdDist_m']))
                    remove_filter = np.logical_or(remove_filter, arrPoint[:, 0:1] < float(self.config.PARM_PC['MinThresholdX_m']))
                    remove_filter = np.logical_or(remove_filter, arrPoint[:, 0:1] > float(self.config.PARM_PC['MaxThresholdX_m']))
                    remove_filter = np.logical_or(remove_filter, arrPoint[:, 1:2] < float(self.config.PARM_PC['MinThresholdY_m']))
                    remove_filter = np.logical_or(remove_filter, arrPoint[:, 1:2] > float(self.config.PARM_PC['MaxThresholdY_m']))
                    remove_filter = np.logical_or(remove_filter, arrPoint[:, 2:3] < float(self.config.PARM_PC['MinThresholdZ_m']))
                    remove_filter = np.logical_or(remove_filter, arrPoint[:, 2:3] > float(self.config.PARM_PC['MaxThresholdZ_m']))

                    # filtering
                    FilteredPointCloud = np.ma.compress(np.bitwise_not(np.ravel(remove_filter)), arrPoint[:, 0:3], axis=0)

                    # Save point cloud
                    PointCloudList[i] = np.array(FilteredPointCloud)
                    pointcloud_timestamp.append(df_pointcloud['timestamp'].values[i])
                    pointcloud_index.append(i)

                    index = index + 1


                # Add point cloud of one LIDAR to self.PointCloudSensorList
                self.PointCloudSensorList[idxSensor] = PointCloudList

                # Generate data frame
                pointcloud_data = {'timestamp': pointcloud_timestamp, 'PointCloud_' + str(idxSensor): pointcloud_index}
                df_pointcloud_idx = pd.DataFrame(pointcloud_data)
                df_pointcloud_idx = df_pointcloud_idx.set_index('timestamp')
                self.df_info = pd.concat([self.df_info, df_pointcloud_idx], axis=1)

                p_index = p_index + 1.0

            self.text_pointcloud = df_pointcloud
            # -----------------------------------------------------------------------------------------------------------------------------
            # 2-2. Resample time
            # -----------------------------------------------------------------------------------------------------------------------------
            # Interpolation
            for i in list(self.df_info.columns.values):
                if i[-1].isdigit():
                    self.df_info[i].fillna(0, inplace=True)
            self.df_info = self.df_info.interpolate(method='linear')  # Linear interpolation in NaN
            self.df_info = self.df_info.dropna(how='any')  # not interpolated rows dropped

            # Remove rows without num_points
            valid_info = []
            for i in list(self.df_info.columns.values):
                if i[-1].isdigit():
                    if len(valid_info) == 0:
                        valid_info = self.df_info[i].values != 0
                    else:
                        valid_info = valid_info | (self.df_info[i].values != 0)
            non_valid_info = ~valid_info
            self.df_info = self.df_info.drop(self.df_info[non_valid_info].index)
            del valid_info, non_valid_info

            # -----------------------------------------------------------------------------------------------------------------------------
            # 2-3. Limit time data
            # -----------------------------------------------------------------------------------------------------------------------------

            # Set using time
            self.DefaultStartTime = self.df_info.index.values[1]
            self.DefaultEndTime = self.df_info.index.values[len(self.df_info.index.values) - 1]


            thread.msleep(1)
            thread.mutex.unlock()
        print('Parse pointcloud logging data')