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
        
        # roll, pitch, z
        self.calib_result = [0.0, 0.0, 0.0]
        self.mean_distance = [0.0]
        self.ground_slope = 0
        self.timestamp = 0
        
        # Path and file
        self.export_path = ''
        self.result_calibration_config_file = ''
        
        # Parameter
        self.CalibrationParam = {}

    def DisplayGroundPoint(self, thread, args):
        thread.emit_string.emit(str('Start display ground point'))
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

            self.timestamp = df_info[strColIndex].index
            self.pointcloud = pointcloud_in_lidar_frame
            self.filtered_pointcloud = filtered_pointcloud_in_lidar_frame

        thread.mutex.unlock()

    def Calibration(self, thread, args):
        thread.emit_string.emit(str('Start z, roll, pitch calibration'))
        thread.mutex.lock()
        has_error = False

        start_time = args[0]
        end_time = args[1]
        PARM_ZRP = copy.deepcopy(args[2])
        plane_method = args[3]
        est_method = args[4]
        selected_sensor = args[5]
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
        filtered_pointcloud_list = []

        dNumOfPointCloudData = len(df_info)

        iteration_size = len(list(range(dNumOfPointCloudData)))
        index = 0
        for idxDataNum in list(range(dNumOfPointCloudData)):
            iteration_ratio = float(index) / float(iteration_size)
            iteration_percentage = iteration_ratio * 100
            thread.change_value.emit(int(iteration_percentage))

            while thread.pause:
                thread.sleep(1)

            if not thread._status:
                break

            strColIndex = 'XYZRGB_' + str(selected_sensor)

            pointcloud_in_lidar_frame = self.importing.PointCloudSensorList[selected_sensor][int(df_info[strColIndex].values[idxDataNum])]
            if len(pointcloud_in_lidar_frame) == 0:
                index += 1
                continue
            pointclouda_in_lidar_frame_homogeneous = np.insert(pointcloud_in_lidar_frame, 3, 1, axis = 1)

            cond = (dXMinThreshold_m < pointcloud_in_lidar_frame[:,0]) & (pointcloud_in_lidar_frame[:,0] < dXMaxThreshold_m) & (dYMinThreshold_m < pointcloud_in_lidar_frame[:,1]) & (pointcloud_in_lidar_frame[:,1] < dYMaxThreshold_m) & (dZMinThreshold_m < pointcloud_in_lidar_frame[:,2]) & (pointcloud_in_lidar_frame[:,2] < dZMaxThreshold_m)

            filtered_pointcloud_in_lidar_frame = pointcloud_in_lidar_frame[cond]
            filtered_pointcloud_in_lidar_frame_homogeneous = np.insert(filtered_pointcloud_in_lidar_frame, 3, 1, axis = 1)
            filtered_pointcloud_list.append(filtered_pointcloud_in_lidar_frame_homogeneous)
            self.timestamp = df_info[strColIndex].index

            if len(filtered_pointcloud_in_lidar_frame_homogeneous) < 3:
                print('There are empty points in ROI')
                thread.emit_string.emit(str('There are empty points in ROI'))
                has_error = True
                break

            if plane_method == 1:
                SVD = utils_plane.fitPlaneSVD(filtered_pointcloud_in_lidar_frame)
                if SVD[2] < 0:
                    SVD = -SVD
                Plane_Norm = np.insert(SVD, 3, filtered_pointcloud_in_lidar_frame[0,0]*SVD[0] + filtered_pointcloud_in_lidar_frame[0,1]*SVD[1] + filtered_pointcloud_in_lidar_frame[0,2]*SVD[2])
     
            elif plane_method == 2:
                LTSQ = utils_plane.fitPLaneLTSQ(filtered_pointcloud_in_lidar_frame)
                if LTSQ[2] < 0:
                    LTSQ = -LTSQ
                Plane_Norm = np.insert(LTSQ, 3, filtered_pointcloud_in_lidar_frame[0,0]*LTSQ[0] + filtered_pointcloud_in_lidar_frame[0,1]*LTSQ[1] + filtered_pointcloud_in_lidar_frame[0,2]*LTSQ[2])
            
            elif plane_method == 3:
                RANSAC = np.asarray(utils_plane.find_plane(filtered_pointcloud_in_lidar_frame))
                if RANSAC[2] < 0:
                    RANSAC = -RANSAC
                value = filtered_pointcloud_in_lidar_frame[0,0]*RANSAC[0] + filtered_pointcloud_in_lidar_frame[0,1]*RANSAC[1] + filtered_pointcloud_in_lidar_frame[0,2]*RANSAC[2]
                Plane_Norm = np.insert(RANSAC, 3, value)

            roll_rad = -1*math.atan2(-Plane_Norm[1],Plane_Norm[2])
            pitch_rad = -1*math.asin(Plane_Norm[0])
    
            roll_deg = roll_rad*180/np.pi
            pitch_deg = pitch_rad*180/np.pi
            
            measured_roll = roll_rad
            measured_pitch = pitch_rad
            array = Plane_Norm

            Distance_to_plane = []

            for idx in list(range(len(filtered_pointcloud_in_lidar_frame))):
                a = array[0]
                b = array[1]
                c = array[2]
                d = -1*array[3]
                x = filtered_pointcloud_in_lidar_frame[idx,0]
                y = filtered_pointcloud_in_lidar_frame[idx,1]
                z = filtered_pointcloud_in_lidar_frame[idx,2]

                distance = np.abs(a*x + b*y + c*z + d)/np.sqrt(a*a + b*b + c*c)
                Distance_to_plane.append(distance)

            iteration_ratio = float(index) / float(iteration_size)
            iteration_percentage = iteration_ratio * 100

            measured_distance_m.append(np.mean(Distance_to_plane))

            Rotated_tf = np.array([[np.cos(measured_pitch), np.sin(measured_pitch)*np.sin(measured_roll), np.sin(measured_pitch)*np.cos(measured_roll), 0.],
                                   [0, np.cos(measured_roll), -1*np.sin(measured_roll), 0.],
                                   [-1*np.sin(measured_pitch), np.cos(measured_pitch)*np.sin(measured_roll), np.cos(measured_pitch)*np.cos(measured_roll), 0.],
                                   [0., 0., 0., 1.]])
            Rotated_pc = np.matmul(Rotated_tf, np.transpose(filtered_pointcloud_in_lidar_frame_homogeneous))

            measured_height = -1*Rotated_pc[2,:]
            measured_height = np.mean(measured_height)

            #if np.abs(dInitRoll_deg - measure d_roll*180/np.pi) < dRollThreshold_deg & np.abs(dInitPitch_deg - measured_pitch*180/np.pi) < dPitchThreshold_deg
            measured_roll_deg.append(measured_roll*180/np.pi)
            measured_pitch_deg.append(measured_pitch*180/np.pi)
            measured_height_m.append(measured_height)

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

            index += 1

        iteration_ratio = float(index) / float(iteration_size)
        iteration_percentage = iteration_ratio * 100
        if iteration_percentage >= 99.8:
            iteration_percentage = 100.0
        thread.change_value.emit(int(iteration_percentage))

        if not thread._status:
            thread.emit_string.emit(str("Stop lidar {} Z, Roll, Pitch Calibrations".format(selected_sensor)))

        elif has_error:
            self.calib_result = [0.0, 0.0, 0.0]
            thread.emit_string.emit('Interrupted lidar {} calibration'.format(selected_sensor))

        elif not has_error:
            # roll, pitch, height calibration
            if est_method == 1:
                calib_roll_result = np.mean(measured_roll_deg) * np.pi/180
                calib_pitch_result = np.mean(measured_pitch_deg) * np.pi/180
                calib_height_result = np.mean(measured_height_m)

            elif est_method == 2:
                calib_roll_result = roll_rls[len(roll_rls)-1]
                calib_pitch_result = pitch_rls[len(pitch_rls)-1]
                calib_height_result = height_rls[len(height_rls)-1]

            Transformation_veh2lidar = np.array([[np.cos(calib_pitch_result), np.sin(calib_pitch_result)*np.sin(calib_roll_result), np.sin(calib_pitch_result)*np.cos(calib_roll_result), 0.],
                        [0, np.cos(calib_roll_result), -1*np.sin(calib_roll_result), 0.],
                        [-1*np.sin(calib_pitch_result), np.cos(calib_pitch_result)*np.sin(calib_roll_result), np.cos(calib_pitch_result)*np.cos(calib_roll_result), calib_height_result],
                        [0., 0., 0., 1.]])
            
            slope_list = []
            for i in list(range(len(filtered_pointcloud_list))):

                filtered_pointcloud_in_veh_frame = np.matmul(Transformation_veh2lidar, np.transpose(filtered_pointcloud_list[i]))
    
                filtered_pointcloud_in_veh_frame = np.transpose(filtered_pointcloud_in_veh_frame)
    
                SVD_veh = utils_plane.fitPlaneSVD(filtered_pointcloud_in_veh_frame)
                if SVD_veh[2] < 0:
                    SVD_veh = -SVD_veh
    
                x1 = SVD_veh[0]
                y1 = SVD_veh[1]
                z1 = SVD_veh[2]
    
                x2 = 0
                y2 = 0
                z2 = 1

                theta = np.arccos((x1*x2+y1*y2+z1*z2)/(np.sqrt(x1*x1 + y1*y1 + z1*z1)*np.sqrt(x2*x2 + y2*y2 + z2*z2)))
    
                min_theta = np.minimum(theta, np.pi-theta)*180/np.pi
                
                slope_list.append(min_theta)
                
            roll_deg = calib_roll_result * 180/np.pi
            pitch_deg = calib_pitch_result * 180/np.pi
            z_m = calib_height_result

            print("Complete lidar {} Z, Roll, Pitch Calibrations".format(selected_sensor))
            self.calib_result = [z_m, roll_deg, pitch_deg]
            self.mean_distance = measured_distance_m
            #self.ground_slope = min_theta
            self.ground_slope = slope_list
            thread.emit_string.emit(str("Complete lidar {} Z, Roll, Pitch Calibrations".format(selected_sensor)))

        thread.mutex.unlock()
