# -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
@date: 2018-07-12
@version: 0.0.1
"""

# Basic modules in Anaconda
import numpy as np
import pandas as pd
# Additional modules
from tqdm import tqdm
import copy

# User defined modules
from applications.utils import utils_pose, utils_pose_dr, utils_file


class Import:
    def __init__(self, config):
        self.config = config

        # Path and file
        self.gnss_logging_file = ''
        self.point_cloud_logging_path = ''

        # Parameter
        self.DefaultStartTime = 0.0
        self.DefaultEndTime = 0.0
        self.PointCloudSensorList = {}
        self.has_gnss_file = False
        self.has_motion_file = False
        self.is_complete = False
        self.df_gnss = None
        self.init = [0., 0., 0.]
        self.df_motion = None
        self.df_motion_input = None
        self.df_pointcloud_idx = {}

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
        self.df_gnss = utils_file.parse_gnss_csv_df(gnss_logging_file, gnss_usecols)

        # Add ENU coordinate
        RefWGS84 = [self.df_gnss['latitude'][0], self.df_gnss['longitude'][0]]
        self.df_gnss = utils_pose.get_gnss_enu(self.df_gnss, RefWGS84)
        self.df_gnss = self.df_gnss.set_index('timestamp')
        self.df_gnss = self.df_gnss[~self.df_gnss.index.duplicated()]
        self.df_gnss = self.df_gnss.dropna(how='any')
        self.df_info = self.df_gnss
        self.default_df_info = copy.deepcopy(self.df_info)
        # self.dr_info : ['timestamp', 'east_m', 'north_m', 'heading']
        print('Parse Gnss logging data')

    def ParseMotion(self):
        motion_usecols = ['timestamp', 'speed_x', 'yaw_rate']  # select 'timestamp','speed_x','yaw_rate'as using
        motion_file = self.gnss_logging_file + '/Motion.csv'
        # motion file motion usecols ???????????? ?????? motion file parsing
        self.df_motion_input = utils_file.parse_motion_csv_df(motion_file, motion_usecols)
        # self.df_motion : [index, 'timestamp', 'speed_x', 'yaw_rate']

        self.df_motion_input = self.df_motion_input.set_index('timestamp')
        self.df_motion_input = self.df_motion_input[~self.df_motion_input.index.duplicated()]
        self.df_motion_input['yaw_rate'] = self.df_motion_input['yaw_rate'] * 180 / np.pi  # rad 2 deg

        try:
            self.init = [self.df_gnss['east_m'].values[0], self.df_gnss['north_m'].values[0],
                         self.df_gnss['heading'].values[0]]  # Dead Reckoning??? ????????? df_gnss??? ??????????????? ??????
            df_motion = copy.deepcopy(self.df_motion_input.dropna(how='any'))
            df_motion = utils_pose_dr.get_motion_enu(df_motion, self.init)  # ????????? ????????? ???????????? ???????????? DeadReckoning ??????
            # df_motion : ['timestamp', 'speed_x', 'yaw_rate', 'dr_east_m', 'dr_north_m', 'dr_heading']

            self.df_info = pd.concat([self.df_gnss, df_motion], axis=1)
            self.default_df_info = copy.deepcopy(self.df_info)
        except:
            df_motion = copy.deepcopy(self.df_motion_input.dropna(how='any'))
            df_motion = utils_pose_dr.get_motion_enu(df_motion, self.init)  # ????????? ????????? ???????????? ???????????? DeadReckoning ??????
            # df_motion : ['timestamp', 'speed_x', 'yaw_rate', 'dr_east_m', 'dr_north_m', 'dr_heading']

            self.df_info = df_motion
            self.default_df_info = copy.deepcopy(self.df_info)

        print('Parse Motion logging data')

    def ParsePointCloud(self, thread):
        thread.mutex.lock()
        # Parse Point Cloud
        PointCloudFileCnt = 0
        self.PointCloudSensorList = {}

        lidar_len = len(self.config.PARM_LIDAR['CheckedSensorList'])
        p_index = 0.0
        for idxSensor in self.config.PARM_LIDAR['CheckedSensorList']:
            pointcloud_file = self.point_cloud_logging_path + '/XYZRGB_' + str(idxSensor) + '.bin'
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
                iteration_ratio = float(index) / float(iteration_size)
                iteration_percentage = iteration_ratio * 100

                epoch_ratio = (iteration_ratio + p_index) / float(lidar_len)
                epoch_percentage = epoch_ratio * 100

                thread.iteration_percentage.emit({idxSensor: iteration_percentage})
                thread.change_value.emit(int(epoch_percentage))

                if df_pointcloud['num_points'].values[i] == 0:
                    index += 1
                    continue

                # Get point cloud
                arrPoint = utils_file.get_point_cloud(pointcloud_file, df_pointcloud['num_points'].values[i],
                                                      df_pointcloud['file_pointer'].values[i])

                tmpXYZ = np.sqrt(
                    np.power(arrPoint[:, 0:1], 2) + np.power(arrPoint[:, 1:2], 2) + np.power(arrPoint[:, 2:3], 2))

                # Check parameter
                remove_filter = tmpXYZ < float(self.config.PARM_PC['MinThresholdDist_m'])
                remove_filter = np.logical_or(remove_filter, tmpXYZ > float(self.config.PARM_PC['MaxThresholdDist_m']))
                remove_filter = np.logical_or(remove_filter,
                                              arrPoint[:, 0:1] < float(self.config.PARM_PC['MinThresholdX_m']))
                remove_filter = np.logical_or(remove_filter,
                                              arrPoint[:, 0:1] > float(self.config.PARM_PC['MaxThresholdX_m']))
                remove_filter = np.logical_or(remove_filter,
                                              arrPoint[:, 1:2] < float(self.config.PARM_PC['MinThresholdY_m']))
                remove_filter = np.logical_or(remove_filter,
                                              arrPoint[:, 1:2] > float(self.config.PARM_PC['MaxThresholdY_m']))
                remove_filter = np.logical_or(remove_filter,
                                              arrPoint[:, 2:3] < float(self.config.PARM_PC['MinThresholdZ_m']))
                remove_filter = np.logical_or(remove_filter,
                                              arrPoint[:, 2:3] > float(self.config.PARM_PC['MaxThresholdZ_m']))
                remove_filter = np.logical_or(remove_filter,
                                              np.isnan(arrPoint[:, 0:1]))
                remove_filter = np.logical_or(remove_filter,
                                              np.isnan(arrPoint[:, 1:2]))
                remove_filter = np.logical_or(remove_filter,
                                              np.isnan(arrPoint[:, 2:3]))

                # filtering
                FilteredPointCloud = np.ma.compress(np.bitwise_not(np.ravel(remove_filter)), arrPoint[:, 0:3], axis=0)

                # Save point cloud
                PointCloudList[i] = np.array(FilteredPointCloud)
                pointcloud_timestamp.append(df_pointcloud['timestamp'].values[i])
                pointcloud_index.append(i)

                index = index + 1

            iteration_ratio = float(index) / float(iteration_size)
            iteration_percentage = iteration_ratio * 100

            if iteration_percentage >= 99.8:
                iteration_percentage = 100.0

            thread.iteration_percentage.emit({idxSensor: iteration_percentage})

            # Add point cloud of one LIDAR to self.PointCloudSensorList
            self.PointCloudSensorList[idxSensor] = PointCloudList

            # Generate data frame
            pointcloud_data = {'timestamp': pointcloud_timestamp, 'XYZRGB_' + str(idxSensor): pointcloud_index}
            self.df_pointcloud_idx[idxSensor] = pd.DataFrame(pointcloud_data)
            self.df_pointcloud_idx[idxSensor] = self.df_pointcloud_idx[idxSensor].set_index('timestamp')
            self.df_pointcloud_idx[idxSensor] = self.df_pointcloud_idx[idxSensor][
                ~self.df_pointcloud_idx[idxSensor].index.duplicated()]
            self.df_info = pd.concat([self.df_info, self.df_pointcloud_idx[idxSensor]], axis=1)
            self.default_df_info = copy.deepcopy(self.df_info)

            p_index = p_index + 1.0

            self.is_complete = True

        epoch_ratio = (iteration_ratio + p_index) / float(lidar_len)
        epoch_percentage = epoch_ratio * 100

        if epoch_percentage >= 99.8:
            epoch_percentage = 100.0

        thread.change_value.emit(int(epoch_percentage))

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
        self.default_df_info = copy.deepcopy(self.df_info)

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
        self.default_df_info = copy.deepcopy(self.df_info)

        del valid_info, non_valid_info

        # -----------------------------------------------------------------------------------------------------------------------------
        # 2-3. Limit time data
        # -----------------------------------------------------------------------------------------------------------------------------

        # Set using time
        self.DefaultStartTime = self.df_info.index.values[1]
        self.DefaultEndTime = self.df_info.index.values[len(self.df_info.index.values) - 1]

        thread.msleep(1)
        thread.mutex.unlock()

    def ChangeInitGnss(self, init):
        self.init = init
        df_gnss = self.default_df_info.dropna(how='any')

        # init_heading = df_gnss['heading'].values[0] + 90 - init[2]
        init_heading = df_gnss['heading'].values[0] + 90 - init[2]

        tf = np.array([[np.cos(-init_heading*np.pi/180.0), -np.sin(-init_heading*np.pi/180.0), init[0]],
                       [np.sin(-init_heading*np.pi/180.0), np.cos(-init_heading*np.pi/180.0), init[1]],
                       [0., 0., 1.]])


        for i in list(range(len(df_gnss['east_m']))):
            tf_state = np.matmul(tf,np.transpose([df_gnss['east_m'].values[i], df_gnss['north_m'].values[i], 1]))

            self.df_info['east_m'].values[i] = tf_state[0]
            self.df_info['north_m'].values[i] = tf_state[1]
            self.df_info['heading'].values[i] = df_gnss['heading'].values[i] - init_heading

    def ChangeInitMotion(self, init):
        self.init = init
        init_value = copy.deepcopy(self.init)
        init_value[2] = init_value[2] - 90
        df_motion = copy.deepcopy(self.df_motion_input.dropna(how='any'))
        df_motion = utils_pose_dr.get_motion_enu(df_motion, init_value)
        # df_motion : ['timestamp', 'speed_x', 'yaw_rate', 'dr_east_m', 'dr_north_m', 'dr_heading']

        self.df_info['dr_east_m'] = df_motion['dr_east_m']
        self.df_info['dr_north_m'] = df_motion['dr_north_m']
        self.df_info['dr_heading'] = df_motion['dr_heading']

    def InitValue(self):
        # for idxSensor in self.config.PARM_LIDAR['CheckedSensorList']:
        #     self.df_info = pd.concat([self.df_info, self.df_pointcloud_idx[idxSensor]], axis=1)

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

    def Clear(self):
        self.DefaultStartTime = 0.0
        self.DefaultEndTime = 0.0
        self.PointCloudSensorList = {}
        self.has_gnss_file = False
        self.has_motion_file = False
        self.is_complete = False
        self.df_gnss = None
        self.init = [0., 0., 0.]
        self.df_motion = None
        self.df_pointcloud_idx = {}