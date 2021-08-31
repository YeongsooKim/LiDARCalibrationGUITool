# -*- coding: utf-8 -*-
"""
@author: yondoo20@gmail.com
@date: 2021-03-18
@version: 0.0.1
"""


# Basic modules in Anaconda
import numpy as np

# Additional modules
from tqdm import tqdm

# User defined modules
from applications.utils import utils_icp
import copy

class DataValidation:
    def __init__(self, config, importing):
        self.config = config
        self.importing = importing
        self.complete_validation = False

        # Path and file
        self.export_path = ''
        self.result_calibration_config_file = ''

        # Parameter
        self.CalibrationParam = {}
        self.TranslationError = {}
        self.RotationError = {}
        self.VehicleTranslation = {}
        self.VehicleRotation = {}
        self.LiDARTranslation = {}
        self.LiDARRotation = {}
        self.RMSERotationErrorDict= {}
        self.RMSETranslationErrorDict = {}
        self.timestamp = 0

        

    def NormAngle_deg(self,dAngle_deg):
        # Set the input angle into the 0~pi
        while (dAngle_deg > 180.0):
            dAngle_deg -= 180.0*2.

        while (dAngle_deg < -180.0):
            dAngle_deg += 180.0*2.

        return dAngle_deg
        
    def Validation(self, thread, args):
        thread.emit_string.emit(str('Start data validation'))
        thread.mutex.lock()
        start_time = args[0]
        end_time = args[1]
        PARM_LIDAR = copy.deepcopy(args[2])
        using_motion_data = args[3]
        vehicle_speed_threshold = args[4] / 3.6
        zrp_calib = args[5]
        max_thresh_z_m = args[6]
        min_thresh_z_m = args[7]
        df_info = copy.deepcopy(self.importing.df_info)

        # Limit time
        df_info = df_info.drop(
            df_info[(df_info.index < start_time) | (df_info.index > end_time)].index)

        if using_motion_data:
            df_info = df_info.drop(df_info[df_info['speed_x'] < vehicle_speed_threshold].index)


        # -----------------------------------------------------------------------------------------------------------------------------
        # 3-1. Match the point cloud based on ICP
        # -----------------------------------------------------------------------------------------------------------------------------
        diff_point_xyzdh_dict = {}
        diff_gnss_xyzdh_dict = {}
        lidar_len = len(PARM_LIDAR['CheckedSensorList'])
        p_index = 0.0
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            veh_translation = []
            veh_rotation = []
            lidar_translation = []
            lidar_rotation = []
            translation_error = []
            rotation_error = []
            self.RMSETranslationError = []
            self.RMSERotationError = []

            calibrated_point = []

            # Remove rows by other sensors
            strColIndex = 'XYZRGB_' + str(idxSensor)

            ## Calibration Initialization            
            dZ_m = zrp_calib[idxSensor][0]
            dRoll_rad = zrp_calib[idxSensor][1] * np.pi / 180.0
            dPitch_rad = zrp_calib[idxSensor][2] * np.pi / 180.0
            
            tf_RollPitchCalib = np.array([[np.cos(dPitch_rad), np.sin(dPitch_rad)*np.sin(dRoll_rad), np.sin(dPitch_rad)*np.cos(dRoll_rad), 0.],
                                  [0., np.cos(dRoll_rad), -1*np.sin(dRoll_rad), 0.],
                                  [-1*np.sin(dPitch_rad), np.cos(dPitch_rad)*np.sin(dRoll_rad), np.cos(dPitch_rad)*np.cos(dRoll_rad), dZ_m],
                                  [0., 0., 0., 1.]])   
            if not using_motion_data:
                df_one_info = df_info[['east_m', 'north_m', 'heading', strColIndex]]
            elif using_motion_data:
                df_one_info = df_info[['dr_east_m', 'dr_north_m', 'dr_heading', strColIndex]]
                df_one_info.rename(columns={"dr_east_m" : "east_m", "dr_north_m" : "north_m", "dr_heading" : "heading"}, inplace=True)

            df_one_info = df_one_info.drop(df_info[(df_one_info[strColIndex].values == 0)].index)

            # Sampling based on interval
            df_sampled_info = df_one_info.iloc[::self.config.PARM_IM['SamplingInterval'], :]

            # Generate Index Pairs
            idx_pair = []
            l = list(range(len(df_sampled_info)))

            for first, second in zip(l, l[1:]):
                idx_pair.append((first, second))

            # Compute ICP
            pbar = tqdm(idx_pair)
            iteration_size = len(idx_pair)
            index = 0
            time = []
            for i, j in pbar:
                ## Set Progress bar
                while thread.pause:
                    thread.sleep(1)
                # Display progress
                iteration_ratio = float(index + 1) / float(iteration_size)
                iteration_percentage = iteration_ratio * 100

                epoch_ratio = (iteration_ratio + p_index)/float(lidar_len)
                epoch_percentage = epoch_ratio*100

                if epoch_percentage >= 99.8:
                    epoch_percentage = 100.0
                thread.change_value.emit(int(epoch_percentage))

                pbar.set_description("XYZRGB_" + str(idxSensor))

                ## Get point clouds
                pointcloud1_lidar = self.importing.PointCloudSensorList[idxSensor][int(df_sampled_info[strColIndex].values[i])]
                pointcloud2_lidar = self.importing.PointCloudSensorList[idxSensor][int(df_sampled_info[strColIndex].values[j])]

                pointcloud1_lidar_homogeneous = np.insert(pointcloud1_lidar, 3, 1, axis = 1)
                pointcloud2_lidar_homogeneous = np.insert(pointcloud2_lidar, 3, 1, axis = 1)

                # PointCloud Conversion: Roll, Pitch, Height
                pointcloud1_in_lidar_frame_calibrated_rollpitch = np.matmul(tf_RollPitchCalib, np.transpose(pointcloud1_lidar_homogeneous))
                pointcloud1_in_lidar_frame_calibrated_rollpitch = np.delete(pointcloud1_in_lidar_frame_calibrated_rollpitch, 3, axis = 0)
                pointcloud1_in_lidar_frame_calibrated_rollpitch = np.transpose(pointcloud1_in_lidar_frame_calibrated_rollpitch)

                pointcloud2_in_lidar_frame_calibrated_rollpitch = np.matmul(tf_RollPitchCalib, np.transpose(pointcloud2_lidar_homogeneous))
                pointcloud2_in_lidar_frame_calibrated_rollpitch = np.delete(pointcloud2_in_lidar_frame_calibrated_rollpitch, 3, axis = 0)
                pointcloud2_in_lidar_frame_calibrated_rollpitch = np.transpose(pointcloud2_in_lidar_frame_calibrated_rollpitch)

                pointcloud1 = pointcloud1_in_lidar_frame_calibrated_rollpitch
                pointcloud2 = pointcloud2_in_lidar_frame_calibrated_rollpitch

                remove_filter1 = pointcloud1[:, 2] < float(min_thresh_z_m)
                remove_filter1 = np.logical_or(remove_filter1, pointcloud1[:, 2] > float(max_thresh_z_m))

                remove_filter2 = pointcloud2[:, 2] < float(min_thresh_z_m)
                remove_filter2 = np.logical_or(remove_filter2, pointcloud2[:, 2] > float(max_thresh_z_m))

                filtered_pointcloud1_lidar = np.ma.compress(np.bitwise_not(np.ravel(remove_filter1)),
                                                            pointcloud1[:, 0:3], axis=0)
                pointcloud1 = np.array(filtered_pointcloud1_lidar)
                filtered_pointcloud2_lidar = np.ma.compress(np.bitwise_not(np.ravel(remove_filter2)),
                                                            pointcloud2[:, 0:3], axis=0)
                pointcloud2 = np.array(filtered_pointcloud2_lidar)


                calibrated_point.append(pointcloud1)
				
                if pointcloud1.shape[0] < 1:
                    index += 1
                    continue
                if pointcloud2.shape[0] < 1:
                    index += 1
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
                                                                                     tolerance=self.config.PARM_DV['Tolerance'],
                                                                                     max_iterations=self.config.PARM_DV['MaximumIteration'],
                                                                                     rm_outlier_dist=self.config.PARM_DV['OutlierDistance_m'])

                #        # Check Matching of ICP
                #        uf.FnDisplayTransformPointCloud(pointcloud2, pointcloud1, transform_point)

                if converged == False:
                    index += 1
                    continue


                veh_east = transform_gnss[0,3]
                veh_north = transform_gnss[1,3]
                veh_yaw = np.arctan2(transform_gnss[1,0],transform_gnss[0,0]) * 180/np.pi
                veh_yaw = self.NormAngle_deg(veh_yaw)
                v_translation = np.square(veh_east*veh_east + veh_north*veh_north)
                v_rotation = np.square(veh_yaw*veh_yaw)
        
                lidar_east = transform_point[0,3]
                lidar_north = transform_point[1,3]
                lidar_yaw = np.arctan2(transform_point[1,0],transform_point[0,0])*180/np.pi # deg
                lidar_yaw = self.NormAngle_deg(lidar_yaw)
                l_translation = np.square(lidar_east*lidar_east + lidar_north*lidar_north)
                l_rotation = np.square(lidar_yaw*lidar_yaw)
                
                err_translation = np.abs(v_translation - l_translation)
                err_rotation = np.abs(v_rotation - l_rotation)

                # Check validation using parameter
                if (float(err_rotation) < self.config.PARM_DV['FilterHeadingThreshold']) & (
                        float(err_translation) < self.config.PARM_DV['FilterDistanceThreshold']):
                    veh_translation.append(v_translation)
                    veh_rotation.append(v_rotation)
                    lidar_translation.append(l_translation)
                    lidar_rotation.append(l_rotation)

                    translation_error.append(err_translation)
                    rotation_error.append(err_rotation)
                    time.append(df_sampled_info.index[i])
                index = index + 1

                if not thread._status:
                    evaluated_lidar = {}
                    not_evaluated_lidar = {}

                    curr_index = PARM_LIDAR['CheckedSensorList'].index(idxSensor)

                    evaluated_lidar['CheckedSensorList'] = []
                    not_evaluated_lidar['CheckedSensorList'] = []

                    for sensor_id in PARM_LIDAR['CheckedSensorList']:
                        if PARM_LIDAR['CheckedSensorList'].index(sensor_id) <= curr_index:
                            evaluated_lidar['CheckedSensorList'].append(sensor_id)
                        else:
                            not_evaluated_lidar['CheckedSensorList'].append(sensor_id)

                    PARM_LIDAR = copy.deepcopy(evaluated_lidar)
                    break

            rmse_translation_error = np.sqrt(np.sum(np.power(translation_error,2))/len(translation_error))
            rmse_rotation_error = np.sqrt(np.sum(np.power(rotation_error,2))/len(rotation_error))
                
            self.VehicleTranslation[idxSensor]=veh_translation
            self.VehicleRotation[idxSensor]=veh_rotation
            self.LiDARTranslation[idxSensor] = lidar_translation
            self.LiDARRotation[idxSensor] = lidar_rotation
            self.TranslationError[idxSensor] = translation_error
            self.RotationError[idxSensor] = rotation_error
            self.RMSETranslationErrorDict[idxSensor] = rmse_translation_error
            self.RMSERotationErrorDict[idxSensor] = rmse_rotation_error
            self.RMSETranslationError.append(rmse_translation_error)
            self.RMSERotationError.append(rmse_rotation_error)
            self.timestamp = time

            p_index = p_index + 1.0

            if not thread._status:
                thread.emit_string.emit('Interrupted evaluating lidar {} calibration'.format(idxSensor))
                break
            thread.emit_string.emit('Complete evaluating lidar {} validation'.format(idxSensor))
            thread.change_value.emit(int(100))

        if not thread._status:
            for idxSensor in not_evaluated_lidar['CheckedSensorList']:
                thread.emit_string.emit('Never evaluating lidar {} calibration'.format(idxSensor))

        self.PARM_LIDAR = copy.deepcopy(PARM_LIDAR)
        thread.mutex.unlock()

        print("Complete Data validation")
