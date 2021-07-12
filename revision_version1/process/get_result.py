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
import copy

from process import utils_pointcloud

    # Paramet
def GetPlotParam(importing, using_gnss_motion, PARM_LIDAR, calibration_param, start_time, end_time):
    ##################
    # Get calibration data
    tmp_df_info = copy.deepcopy(importing.df_info)

    # Limit time
    df_info = tmp_df_info.drop(
        tmp_df_info[(tmp_df_info.index < start_time) | (tmp_df_info.index > end_time)].index)

    accum_pointcloud = {}
    accum_pointcloud_ = {}
    calib_param_ = [0, 0, 0, 0, 0, 0]
    for idxSensor in PARM_LIDAR['CheckedSensorList']:
        calib_param = calibration_param[idxSensor]

        ##################
        # Remove rows by other sensors
        strColIndex = 'XYZRGB_' + str(idxSensor)

        if not using_gnss_motion:
            df_one_info = df_info[['east_m', 'north_m', 'heading', strColIndex]]
        elif using_gnss_motion:
            df_one_info = df_info[['dr_east_m', 'dr_north_m', 'dr_heading', strColIndex]]
            df_one_info.rename(columns={"dr_east_m": "east_m", "dr_north_m": "north_m", "dr_heading": "heading"}, inplace=True)

        df_one_info = df_one_info.drop(df_info[(df_one_info[strColIndex].values == 0)].index)

        ##################
        ##### Arguments
        pose = df_one_info['east_m'].values
        pose = np.vstack([pose, df_one_info['north_m'].values])
        pose = np.vstack([pose, df_one_info['heading'].values * np.pi / 180.])
        pose = np.vstack([pose, df_one_info[strColIndex].values])
        index_pointcloud = pose[3]

        ##################
        # Get Point cloud list
        pointcloud = importing.PointCloudSensorList[idxSensor]

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

    return df_one_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_



