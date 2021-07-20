# -*- coding: utf-8 -*-
"""
@author: yondoo20@gmail.com
@date: 2021-05-09
@version: 0.0.1
"""

##############################################################################################################################
#%% Import libraries
##############################################################################################################################

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm

import math
from mpl_toolkits.mplot3d import Axes3D

# Additional modules
from tqdm import tqdm                                       
import copy

# User defined modules
from applications.utils import utils_plane, utils_file


##############################################################################################################################
#%% 3. Verification - Motion Data
##############################################################################################################################

class ZRollPitch:
    def __init__(self, config, importing):
        self.config = config
        self.importing = importing
        self.complete_zrollpitch = False
        
        # Path and file
        self.export_path = ''
        self.result_calibration_config_file = ''
        
        # Parameter
        self.CalibrationParam = {}

    def DisplayGroundPoint(self, thread, args):
        thread.emit_string.emit(str('Start z, roll, pitch calibration'))
        thread.mutex.lock()
        start_time = args[0]
        end_time = args[1]
        PARM_ZRP = copy.deepcopy(args[2])
        selected_sensor = args[3]
        df_info = copy.deepcopy(self.importing.df_info)

        # Limit time
        df_info = df_info.drop(
            df_info[(df_info.index < start_time) | (df_info.index > end_time)].index)

        dXMaxThreshold_m = PARM_ZRP['MaxDistanceX_m']
        dXMinThreshold_m = PARM_ZRP['MinDistanceX_m']
        dYMaxThreshold_m = PARM_ZRP['MaxDistanceY_m']
        dYMinThreshold_m = PARM_ZRP['MinDistanceY_m']
        dZMaxThreshold_m = PARM_ZRP['MaxDistanceZ_m']
        dZMinThreshold_m = PARM_ZRP['MinDistanceZ_m']

        dNumOfPointCloudData = len(df_info)
        for idxDataNum in list(range(dNumOfPointCloudData)):
            strColIndex = 'XYZRGB_' + str(selected_sensor)

            pointcloud_in_lidar_frame = self.importing.PointCloudSensorList[selected_sensor][
                int(df_info[strColIndex].values[idxDataNum])]

            if len(pointcloud_in_lidar_frame) == 0:
                continue
                
            cond = (dXMinThreshold_m < pointcloud_in_lidar_frame[:, 0]) & (
                        pointcloud_in_lidar_frame[:, 0] < dXMaxThreshold_m) & (
                               dYMinThreshold_m < pointcloud_in_lidar_frame[:, 1]) & (
                               pointcloud_in_lidar_frame[:, 1] < dYMaxThreshold_m) & (
                               dZMinThreshold_m < pointcloud_in_lidar_frame[:, 2]) & (
                               pointcloud_in_lidar_frame[:, 2] < dZMaxThreshold_m)

            filtered_pointcloud_in_lidar_frame = pointcloud_in_lidar_frame[cond]

            self.pointcloud = pointcloud_in_lidar_frame
            self.filtered_pointcloud = filtered_pointcloud_in_lidar_frame

        thread.mutex.unlock()

    def Calibration(self, thread, args):
        thread.emit_string.emit(str('Start z, roll, pitch calibration'))
        thread.mutex.lock()
        start_time = args[0]
        end_time = args[1]
        PARM_ZRP = copy.deepcopy(args[2])
        df_info = copy.deepcopy(self.importing.df_info)

        # Limit time
        df_info = df_info.drop(
            df_info[(df_info.index < start_time) | (df_info.index > end_time)].index)


        dXMaxThreshold_m = PARM_ZRP['Maximum X Distance']
        dXMinThreshold_m = PARM_ZRP['Minimum X Distance']
        dYMaxThreshold_m = PARM_ZRP['Maximum Y Distance']
        dYMinThreshold_m = PARM_ZRP['Minimum Y Distance']
        dZMaxThreshold_m = PARM_ZRP['Maximum Z Distance']
        dZMinThreshold_m = PARM_ZRP['Minimum Z Distance']

        plane_method = 1 #(1:SVD, 2:LTSQ, 3:RANSAC)
        est_method = 1 #(1:Least Square, 2:Recursive Least Square)

        dInitRoll_deg = 0
        dInitRoll_rad = dInitRoll_deg * np.pi / 180
        dInitPitch_deg = 0
        dInitPitch_rad = dInitPitch_deg * np.pi / 180
        dInitHeight_m = 0

        dRollThreshold_deg = 10
        dPitchThreshold_deg = 10

        roll_P_rls = []
        roll_P_rls.append(100)
        roll_rls = []
        roll_rls_deg = []
        roll_rls.append(0)
        roll_R = 50*np.pi/180

        pitch_P_rls = []
        pitch_rls_deg = []
        pitch_P_rls.append(100)
        pitch_rls = []
        pitch_rls.append(0)
        pitch_R = 50*np.pi/180

        height_P_rls = []
        height_P_rls.append(100)
        height_rls = []
        height_rls.append(0)
        height_R = 50*np.pi/180

        measured_roll_deg = []
        measured_pitch_deg = []
        measured_height_m = []
        measured_distance_m = []
        dNumOfPointCloudData = len(df_info)
        f = []
        ff = []
        pointcloud_ = []
        for idxDataNum in list(range(dNumOfPointCloudData)):
            idxSensor = 0
            strColIndex = 'PointCloud_' + str(idxSensor)
            pointcloud_in_lidar_frame = self.importing.PointCloudSensorList[idxSensor][int(df_info[strColIndex].values[idxDataNum])]
            pointclouda_in_lidar_frame_homogeneous = np.insert(pointcloud_in_lidar_frame, 3, 1, axis = 1)

            pointcloud_.append(pointclouda_in_lidar_frame_homogeneous)

            cond = (dXMinThreshold_m < pointcloud_in_lidar_frame[:,0]) & (pointcloud_in_lidar_frame[:,0] < dXMaxThreshold_m) & (dYMinThreshold_m < pointcloud_in_lidar_frame[:,1]) & (pointcloud_in_lidar_frame[:,1] < dYMaxThreshold_m) & (dZMinThreshold_m < pointcloud_in_lidar_frame[:,2]) & (pointcloud_in_lidar_frame[:,2] < dZMaxThreshold_m)

            filtered_pointclouda_in_lidar_frame = pointcloud_in_lidar_frame[cond]
            filtered_pointclouda_in_lidar_frame_homogeneous = np.insert(filtered_pointclouda_in_lidar_frame, 3, 1, axis = 1)
            f.append(filtered_pointclouda_in_lidar_frame)
            ff.append(filtered_pointclouda_in_lidar_frame_homogeneous)
            SVD = utils_plane.fitPlaneSVD(filtered_pointclouda_in_lidar_frame)
            LTSQ = utils_plane.fitPLaneLTSQ(filtered_pointclouda_in_lidar_frame)
            RANSAC = np.asarray(utils_plane.find_plane(filtered_pointclouda_in_lidar_frame))

        if SVD[2] < 0:
            SVD = -SVD
        if LTSQ[2] < 0:
            LTSQ = -LTSQ
        if RANSAC[2] < 0:
            RANSAC = -RANSAC

        SVD = np.insert(SVD, 3, filtered_pointclouda_in_lidar_frame[0,0]*SVD[0] + filtered_pointclouda_in_lidar_frame[0,1]*SVD[1] + filtered_pointclouda_in_lidar_frame[0,2]*SVD[2])
        LTSQ = np.insert(LTSQ, 3, filtered_pointclouda_in_lidar_frame[0,0]*LTSQ[0] + filtered_pointclouda_in_lidar_frame[0,1]*LTSQ[1] + filtered_pointclouda_in_lidar_frame[0,2]*LTSQ[2])
        value = filtered_pointclouda_in_lidar_frame[0,0]*RANSAC[0] + filtered_pointclouda_in_lidar_frame[0,1]*RANSAC[1] + filtered_pointclouda_in_lidar_frame[0,2]*RANSAC[2]
        RANSAC = np.insert(RANSAC, 3, value)


        roll_svd = -1*math.atan2(-SVD[1],SVD[2])
        pitch_svd = -1*math.asin(SVD[0])

        roll_ltsq = -1*math.atan2(-LTSQ[1],LTSQ[2])
        pitch_ltsq = -1*math.asin(LTSQ[0])

        roll_ransac = -1*math.atan2(-RANSAC[1],RANSAC[2])
        pitch_ransac = -1*math.asin(RANSAC[0])

        roll_svd_deg = roll_svd*180/np.pi
        pitch_svd_deg = pitch_svd*180/np.pi

        roll_ltsq_deg = roll_ltsq*180/np.pi
        pitch_ltsq_deg = pitch_ltsq*180/np.pi

        roll_ransac_deg = roll_ransac*180/np.pi
        pitch_ransac_deg = pitch_ransac*180/np.pi


        if plane_method == 1:
            measured_roll = roll_svd
            measured_pitch = pitch_svd
            array = SVD

        elif plane_method == 2:
            measured_roll = roll_ltsq
            measured_pitch = pitch_ltsq
            array = LTSQ

        elif plane_method == 3:
            measured_roll = roll_ransac
            measured_pitch = pitch_ransac
            array = RANSAC

        Distance_to_plane = []
        for idx in list(range(len(filtered_pointclouda_in_lidar_frame))):
            a = array[0]
            b = array[1]
            c = array[2]
            d = -1*array[3]
            x = filtered_pointclouda_in_lidar_frame[idx,0]
            y = filtered_pointclouda_in_lidar_frame[idx,1]
            z = filtered_pointclouda_in_lidar_frame[idx,2]

            distance = np.abs(a*x + b*y + c*z + d)/np.sqrt(a*a + b*b + c*c)
            Distance_to_plane.append(distance)

        measured_distance_m.append(np.mean(Distance_to_plane))

        Rotated_tf = np.array([[np.cos(measured_pitch), np.sin(measured_pitch)*np.sin(measured_roll), np.sin(measured_pitch)*np.cos(measured_roll), 0.],
                       [0, np.cos(measured_roll), -1*np.sin(measured_roll), 0.],
                       [-1*np.sin(measured_pitch), np.cos(measured_pitch)*np.sin(measured_roll), np.cos(measured_pitch)*np.cos(measured_roll), 0.],
                       [0., 0., 0., 1.]])
        Rotated_pc = np.matmul(Rotated_tf, np.transpose(filtered_pointclouda_in_lidar_frame_homogeneous))

        measured_height = -1*Rotated_pc[2,:]
        measured_height = np.mean(measured_height)

        measured_roll_deg.append(measured_roll*180/np.pi)
        measured_pitch_deg.append(measured_pitch*180/np.pi)
        measured_height_m.append(measured_height)
        #np.abs(dInitRoll_deg - measure d_roll*180/np.pi) < dRollThreshold_deg & np.abs(dInitPitch_deg - measured_pitch*180/np.pi) < dPitchThreshold_deg

        H = 1

        # roll, pitch, height calibration
        roll_K = roll_P_rls[idxDataNum] * H * (1/(H * roll_P_rls[idxDataNum] * H + roll_R))
        roll_rls.append(roll_rls[idxDataNum] + roll_K * (measured_roll - H * roll_rls[idxDataNum]))
        roll_rls_deg.append(roll_rls[idxDataNum]*180/np.pi)
        roll_P_rls.append((1 - roll_K * H) * roll_P_rls[idxDataNum] * (1/(1 - roll_K * H)) + roll_K * roll_R * (1/roll_K))

        pitch_K = pitch_P_rls[idxDataNum] * H * (1/(H * pitch_P_rls[idxDataNum] * H + pitch_R))
        pitch_rls.append(pitch_rls[idxDataNum] + pitch_K * (measured_pitch - H * pitch_rls[idxDataNum]))
        pitch_rls_deg.append(pitch_rls[idxDataNum]*180/np.pi)
        pitch_P_rls.append((1 - pitch_K * H) * pitch_P_rls[idxDataNum] * (1/(1 - pitch_K * H)) + pitch_K * pitch_R * (1/pitch_K))

        height_K = height_P_rls[idxDataNum] * H * (1/(H * height_P_rls[idxDataNum] * H + height_R))
        height_rls.append(height_rls[idxDataNum] + height_K * (measured_height - H * height_rls[idxDataNum]))
        height_P_rls.append((1 - height_K * H) * height_P_rls[idxDataNum] * (1/(1 - height_K * H)) + height_K * pitch_R * (1/height_K))

        # roll, pitch, height calibration
        if est_method == 1:
            calib_roll_result = np.mean(measured_roll_deg) * np.pi/180
            calib_pitch_result = np.mean(measured_pitch_deg) * np.pi/180
            calib_height_result = np.mean(measured_height_m)
        elif est_method == 2:
            calib_roll_result = roll_rls[len(roll_rls)-1]
            calib_pitch_result = pitch_rls[len(pitch_rls)-1]
            calib_height_result = height_rls[len(height_rls)-1]


        self.roll_deg = calib_roll_result * 180/np.pi
        self.pitch_deg = calib_pitch_result * 180/np.pi
        self.z_m = calib_height_result
        thread.mutex.unlock()

        print("Complete Z, Roll, Pitch Calibrations")
