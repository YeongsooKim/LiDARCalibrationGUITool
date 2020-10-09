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

# User defined modules
from process import utils_icp
from process import utils_file
from process import utils_pointcloud
from process import utils_pose

    # Paramet
def GetPlotParam(config, Import, calibration_param, start_time, end_time):
    ##################
    # Get calibration data
    tmp_df_info = Import.df_info

    # Limit time
    df_info = tmp_df_info.drop(
        tmp_df_info[(tmp_df_info.index < start_time) | (tmp_df_info.index > end_time)].index)

    accum_pointcloud = {}
    accum_pointcloud_ = {}
    calib_param_ = [0, 0, 0, 0, 0, 0]
    for idxSensor in config.PARM_LIDAR['CheckedSensorList']:
        calib_param = calibration_param[idxSensor]

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
        pointcloud = Import.PointCloudSensorList[idxSensor]

        ##################
        # Accumulation of point cloud
        num_pose = pose.shape[1]
        accum_point_enup = np.empty((0, 4))
        accum_point_enup_ = np.empty((0, 4))
        for idx_pose in list(range(0, num_pose, 20)):
            # Convert raw to enu
            point_sensor = pointcloud[int(index_pointcloud[idx_pose])][:, 0:3]
            point_enu = utils_pointcloud.cvt_pointcloud_6dof_sensor_enu(point_sensor, calib_param, pose[0:3, idx_pose])
            point_enu_ = utils_pointcloud.cvt_pointcloud_6dof_sensor_enu(point_sensor, calib_param_, pose[0:3, idx_pose])
            # Add index
            point_enup = np.concatenate((point_enu, np.full((point_enu.shape[0], 1), idx_pose)), axis=1)
            point_enup_ = np.concatenate((point_enu_, np.full((point_enu.shape[0], 1), idx_pose)), axis=1)

            # Accumulate the point cloud
            accum_point_enup = np.vstack([accum_point_enup, point_enup])
            accum_point_enup_ = np.vstack([accum_point_enup_, point_enup_])
        accum_pointcloud[idxSensor] = accum_point_enup
        accum_pointcloud_[idxSensor] = accum_point_enup_

    return df_info, config.PARM_LIDAR['CheckedSensorList'], accum_pointcloud, accum_pointcloud_

    # ##############################################################################################################################
    # # %% 5. Save results
    # ##############################################################################################################################
    # def SaveResult(self):
    #     config_lidar_file = self.export_path + '/handeye_result.ini'
    #
    #     result_config = configparser.ConfigParser()
    #     for idxSensor in self.CalibrationParam:
    #         # Section Name
    #         section_name = 'PointCloud_' + str(idxSensor)
    #
    #         # Save parameter
    #         result_config[section_name] = {}
    #         result_config[section_name]['roll'] = str(self.CalibrationParam[idxSensor][0])
    #         result_config[section_name]['pitch'] = str(self.CalibrationParam[idxSensor][1])
    #         result_config[section_name]['yaw'] = str(self.CalibrationParam[idxSensor][2])
    #         result_config[section_name]['x'] = str(self.CalibrationParam[idxSensor][3])
    #         result_config[section_name]['y'] = str(self.CalibrationParam[idxSensor][4])
    #         result_config[section_name]['z'] = str(self.CalibrationParam[idxSensor][5])
    #
    #     with open(config_lidar_file, 'w') as configfile:
    #         result_config.write(configfile)