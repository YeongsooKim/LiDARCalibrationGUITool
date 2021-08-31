 # -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
@date: 2018-07-12
@version: 0.0.1
"""

import numpy as np
import copy
import open3d as o3d

# User defined modules
from tqdm import tqdm
from applications.utils import utils_icp, utils_file
from applications.utils import utils_pointcloud

class Evaluation:
    def __init__(self, config, importing):
        self.config = config
        self.importing = importing

    def Evaluation(self, thread, args):
        thread.mutex.lock()

        start_time = args[0]
        end_time = args[1]
        PARM_LIDAR = args[2]
        CalibrationParam = copy.deepcopy(args[3])
        using_gnss_motion = args[4]
        zrp_calib = args[5]
        dist_interval = args[6]
        max_thresh_z_m = args[7]
        min_thresh_z_m = args[8]
        voxel_grid_size = args[9]

        # Get calibration data
        tmp_df_info = copy.deepcopy(self.importing.df_info)

        # Limit time
        df_info = tmp_df_info.drop(
            tmp_df_info[(tmp_df_info.index < start_time) | (tmp_df_info.index > end_time)].index)

        init_T = np.array([[np.cos(5 * np.pi / 180), -np.sin(5 * np.pi / 180), 0., 0.5],
                           [np.sin(5 * np.pi / 180), np.cos(5 * np.pi / 180), 0., 0.5],
                           [0., 0., 1., 0.],
                           [0., 0., 0., 1.]])

        Map = {}
        x_sensor_localization = {}
        y_sensor_localization = {}
        yaw_sensor_localization = {}

        ref_sensor_localization = {}
        pred_sensor_localization = {}
        error_sensor_localization = {}



        LiDAR_list = {}
        lidar_len = len(PARM_LIDAR['CheckedSensorList'])
        p_index = 0.0
        for idxSensor in PARM_LIDAR['CheckedSensorList']:  # lidar 개수만큼 for문 반복
            calib = CalibrationParam[idxSensor]

            # Remove rows by other sensors
            dZ_m = zrp_calib[idxSensor][0]
            dRoll_rad = zrp_calib[idxSensor][1] * np.pi / 180.0
            dPitch_rad = zrp_calib[idxSensor][2] * np.pi / 180.0

            tf_RollPitchCalib = np.array([[np.cos(dPitch_rad), np.sin(dPitch_rad)*np.sin(dRoll_rad), np.sin(dPitch_rad)*np.cos(dRoll_rad), 0.],
                                  [0., np.cos(dRoll_rad), -1*np.sin(dRoll_rad), 0.],
                                  [-1*np.sin(dPitch_rad), np.cos(dPitch_rad)*np.sin(dRoll_rad), np.cos(dPitch_rad)*np.cos(dRoll_rad), dZ_m],
                                  [0., 0., 0., 1.]])

            tf_XYYaw = np.array([[np.cos(calib[2]), -np.sin(calib[2]), 0., calib[3]],
                                 [np.sin(calib[2]), np.cos(calib[2]), 0., calib[4]],
                                 [0., 0., 1., 0.],
                                 [0., 0., 0., 1.]])


            # Remove rows by other sensors
            strColIndex = 'XYZRGB_' + str(idxSensor)  # PointCloud_n 이름 생성
            LiDAR_list[idxSensor] = strColIndex

            if using_gnss_motion == 0:
                df_one_info = df_info[['east_m', 'north_m', 'heading', strColIndex]]
            else:
                df_one_info = df_info[['dr_east_m', 'dr_north_m', 'dr_heading', strColIndex]]
                df_one_info.rename(columns={"dr_east_m": "east_m", "dr_north_m": "north_m", "dr_heading": "heading"},
                                   inplace=True)



            df_one_info = df_one_info.drop(df_info[(df_one_info[
                                                        strColIndex].values == 0)].index)  # df_one_info에서 strColIndex(PointCloud_n)의 value가 0인 값들 다 제외 --> 값이 있는 애들만 남겨놓음

            # Sampling based on interval
            df_sampled_info = df_one_info.iloc[::self.config.PARM_EV['SamplingInterval'],
                              :]  # Sampling 간격만큼 분할(추출)함 ex) samplinginterval = 2이면 2행마다 하나 추출
            #df_sampled_info.heading = df_sampled_info.heading + 90

            l = range(len(df_sampled_info)-1)  # df_sampled_info의 길이만큼 range생성 후 list로 변환
            # l = [0,1,2,3,...,len(df_sampled_info)-1]

            ##-----------------------------------------------------------------------------------------------------------------------------
            # 6-1. Generating a "HD map" global point cloud map from RTK-GNSS and init calibration
            ##-----------------------------------------------------------------------------------------------------------------------------
            HD_map = np.empty((0, 3))
            ### Make HD Map
            calib_param = CalibrationParam[idxSensor]
            calib_param[0] = 0.0
            calib_param[1] = 0.0
            calib_param[5] = 0.0

            ##################
            ##### Arguments
            pose = df_sampled_info['east_m'].values
            pose = np.vstack([pose, df_sampled_info['north_m'].values])
            pose = np.vstack([pose, df_sampled_info['heading'].values * np.pi / 180.])

            pointcloud = self.importing.PointCloudSensorList[idxSensor]

            print('\n----------- Start Mapping -----------')
            # HD Map 생성
            pbar = tqdm(l)  # idx_pair progress bar 생성
            iteration_size = len(l)
            thread.emit_string.emit('Map generating LiDAR {}'.format(idxSensor))
            index = 0
            for idx_pose in pbar:  # make hd map

                iteration_ratio = float(index + 1) / float(iteration_size)
                iteration_percentage = iteration_ratio * 100

                epoch_ratio = (iteration_ratio + p_index) / (float(lidar_len) * 10) # job cost of mapping is one and cost of evauation is nine
                epoch_percentage = epoch_ratio * 100

                thread.change_value.emit(int(epoch_percentage))

                pbar.set_description("Progress of Mapping")  # 상태바 naming


                point_sensor = pointcloud[int(df_one_info[strColIndex].values[idx_pose])]
                point_sensor_homogeneous = np.insert(point_sensor, 3, 1, axis=1)

                # PointCloud Conversion: Roll, Pitch, Height
                point_sensor_calibrated_rollpitch = np.matmul(tf_RollPitchCalib,
                                                              np.transpose(point_sensor_homogeneous))
                point_sensor_calibrated_rollpitch = np.transpose(
                    point_sensor_calibrated_rollpitch)

                point_sensor_calibrated_rollpitch = np.delete(
                    point_sensor_calibrated_rollpitch, 3, axis=1)

                point_sensor = point_sensor_calibrated_rollpitch

                remove_filter = point_sensor[:, 2] < float(min_thresh_z_m)
                remove_filter = np.logical_or(remove_filter, point_sensor[:, 2] > float(max_thresh_z_m))

                filtered_point_sensor = np.ma.compress(np.bitwise_not(np.ravel(remove_filter)),
                                                       point_sensor[:, 0:3], axis=0)
                point_sensor = np.array(filtered_point_sensor)

                point_enu = utils_pointcloud.cvt_pointcloud_6dof_sensor_enu(point_sensor, calib_param,
                                                                            pose[0:3, idx_pose])
                # Accumulate the point cloud
                HD_map = np.vstack([HD_map, point_enu])


                index = index + 1

            p_index = p_index + 1.0

            HD_map_pcd = o3d.geometry.PointCloud()
            HD_map_pcd.points = o3d.utility.Vector3dVector(HD_map)

            downsampled_HD_map_pcd = o3d.geometry.voxel_down_sample(HD_map_pcd, voxel_grid_size)  # using voxel grid filter

            # pcd to array
            Map[idxSensor] = np.asarray(downsampled_HD_map_pcd.points)
            #Map[idxSensor] = HD_map


        ##-----------------------------------------------------------------------------------------------------------------------------
        # 6-2. Localization
        ##-----------------------------------------------------------------------------------------------------------------------------
        for idxSensor in PARM_LIDAR['CheckedSensorList']:  # lidar 개수만큼 for문 반복
            ref_each_localization = []
            pred_each_localization = []
            error_each_localization = []
            x_each_localization = []
            y_each_localization = []
            yaw_each_localization = []
            threshold = 30

            HD_map = Map[idxSensor]


            df_one_info = df_info[['east_m', 'north_m', 'heading', 'dr_east_m', 'dr_north_m', 'dr_heading',
                                   strColIndex]]  # df_info의 'east','north','heading',...,XYZRGB 하나씩 분리해서 저장
            df_one_info = df_one_info.drop(df_info[(df_one_info[
                                                        strColIndex].values == 0)].index)  # df_one_info에서 strColIndex(PointCloud_n)의 value가 0인 값들 다 제외 --> 값이 있는 애들만 남겨놓음

            # Sampling based on interval
            df_sampled_info = df_one_info.iloc[::self.config.PARM_EV['SamplingInterval'],
                              :]  # Sampling 간격만큼 분할(추출)함 ex) samplinginterval = 2이면 2행마다 하나 추출
            df_sampled_info.heading = df_sampled_info.heading + 90

            pbar = tqdm(l)  # idx_pair progress bar 생성

            iteration_size = len(l)
            index = 0
            distance= 0
            thread.emit_string.emit('Start Evaluation LiDAR_{}'.format(idxSensor))
            for i in pbar:
                # Display progress
                iteration_ratio = float(index + 1) / float(iteration_size)
                iteration_percentage = iteration_ratio * 100

                epoch_ratio = (9 * iteration_ratio + p_index)/(float(lidar_len)*10)
                epoch_percentage = epoch_ratio*100

                thread.change_value.emit(int(epoch_percentage))

                pbar.set_description("Evaluation" + str(idxSensor))  # 상태바 naming
                if using_gnss_motion == False:
                    ref_east = df_sampled_info['east_m'].values[i]
                    ref_north = df_sampled_info['north_m'].values[i]
                    ref_yaw = df_sampled_info['heading'].values[i]

                    east1 = df_sampled_info['east_m'].values[i]
                    east2 = df_sampled_info['east_m'].values[i + 1]
                    north1 = df_sampled_info['north_m'].values[i]
                    north2 = df_sampled_info['north_m'].values[i + 1]

                if using_gnss_motion == True:
                    ref_east = df_sampled_info['dr_east_m'].values[i]
                    ref_north = df_sampled_info['dr_north_m'].values[i]
                    ref_yaw = df_sampled_info['dr_heading'].values[i]

                    east1 = df_sampled_info['dr_east_m'].values[i]
                    east2 = df_sampled_info['dr_east_m'].values[i + 1]
                    north1 = df_sampled_info['dr_north_m'].values[i]
                    north2 = df_sampled_info['dr_north_m'].values[i + 1]

                ref_yaw = utils_file.normalized_angle_deg(ref_yaw)
                ref_yaw_rad = ref_yaw * np.pi / 180

                tmp_distance = np.sqrt((east2 - east1)*(east2 - east1) + (north2 - north1)*(north2 - north1))

                distance = distance + tmp_distance

                if distance > dist_interval:
                    pointcloud = self.importing.PointCloudSensorList[idxSensor][int(df_sampled_info[strColIndex].values[i])]
                    pointcloud_homogeneous = np.insert(pointcloud, 3, 1, axis=1)

                    # PointCloud Conversion: Roll, Pitch, Height
                    pointcloud_calibrated_rollpitch = np.matmul(tf_RollPitchCalib,
                                                                  np.transpose(pointcloud_homogeneous))
                    pointcloud_calibrated_rollpitch = np.transpose(
                        pointcloud_calibrated_rollpitch)

                    pointcloud_calibrated_rollpitch = np.delete(
                        pointcloud_calibrated_rollpitch, 3, axis=1)

                    pointcloud = pointcloud_calibrated_rollpitch

                    remove_filter = pointcloud[:, 2] < float(min_thresh_z_m)
                    remove_filter = np.logical_or(remove_filter, pointcloud[:, 2] > float(max_thresh_z_m))

                    filtered_pointcloud = np.ma.compress(np.bitwise_not(np.ravel(remove_filter)),
                                                           pointcloud[:, 0:3], axis=0)
                    pointcloud = np.array(filtered_pointcloud)

                    if pointcloud.shape[0] < 1:
                        continue

                    ref_transformation_world_to_veh = np.array(
                        [[np.cos(ref_yaw_rad), -np.sin(ref_yaw_rad), 0., ref_east],
                         [np.sin(ref_yaw_rad), np.cos(ref_yaw_rad), 0., ref_north],
                         [0., 0., 1., 0.],
                         [0., 0., 0., 1.]])

                    pred_transformation = np.matmul(ref_transformation_world_to_veh, tf_XYYaw * init_T)

                    # Update ICP Map Matching
                    # pred_east, pred_north기준으로 threshold만큼 ROI 설정
                    cond = np.sqrt(
                        np.power((HD_map[:, 0] - ref_east), 2) + np.power((HD_map[:, 1] - ref_north), 2)) < threshold
                    map_in_ROI = HD_map[cond]

                    if map_in_ROI.shape[0] <1 :
                        continue

                    # Get Transformation between HD Map in Vehicle Frame and Pointcloud in LiDAR Frame
                    # Map Matching
                    print('\n----------- Start ICP -----------')

                    # ICP Registration: world to lidar
                    transform_point, distances, iterations, converged = utils_icp.icp_NM(pointcloud[:, 0:3],
                                                                                         map_in_ROI[:, 0:3],
                                                                                         init_pose=pred_transformation,
                                                                                         tolerance=0.0001,
                                                                                         max_iterations=100,
                                                                                         rm_outlier_dist=30)

                    print('\n----------- Finish ICP -----------')

                    # Transformation between HD Map and pointcloud in veh frame
                    transformation_result = transform_point
                    # transformation_veh_to_lidar

                    if converged == False:
                        index += 1
                        continue

                    # Get the transformation world to vehicle (원래는 ref_transformation_world_to_veh이 아닌, dr 곱해줘양함)
                    pred_transformation_world_to_veh = np.matmul(transformation_result,
                                                                 np.linalg.inv(tf_XYYaw))

                    pred_east = pred_transformation_world_to_veh[0][3]
                    pred_north = pred_transformation_world_to_veh[1][3]
                    pred_yaw = np.arctan2(pred_transformation_world_to_veh[1, 0], pred_transformation_world_to_veh[0, 0])
                    pred_yaw = utils_file.normalized_angle_rad(pred_yaw)
                    pred_yaw_deg = pred_yaw * 180 / np.pi

                    ref_each_localization.append([ref_east, ref_north, ref_yaw])
                    pred_each_localization.append([pred_east, pred_north, pred_yaw_deg])
                    error_each_localization.append(
                        [ref_east - pred_east, ref_north - pred_north, (ref_yaw_rad - pred_yaw) * 180 / np.pi])
                    x_each_localization.append(ref_east - pred_east)
                    y_each_localization.append(ref_north - pred_north)
                    yaw_each_localization.append((ref_yaw - pred_yaw_deg))

                    distance = 0

                # idx.append(i)
                # str_idx.append(num2str(i))
                index += 1

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
            if thread._status:
                epoch_percentage = 100.0
                thread.change_value.emit(int(epoch_percentage))

            ref_sensor_localization[idxSensor] = ref_each_localization
            pred_sensor_localization[idxSensor] = pred_each_localization
            error_sensor_localization[idxSensor] = error_each_localization
            x_sensor_localization[idxSensor] = x_each_localization
            y_sensor_localization[idxSensor] = y_each_localization
            yaw_sensor_localization[idxSensor] = yaw_each_localization

            p_index = p_index + 9.0
            if not thread._status:
                thread.emit_string.emit('Interrupted evaluating lidar {} calibration'.format(idxSensor))
                break
            thread.emit_string.emit('Complete evaluating lidar {} calibration'.format(idxSensor))

        if not thread._status:
            for idxSensor in not_evaluated_lidar['CheckedSensorList']:
                thread.emit_string.emit('Never evaluating lidar {} calibration'.format(idxSensor))

        RMSE_x = {}
        rmse_x = []
        RMSE_y = {}
        rmse_y = []
        RMSE_yaw = {}
        rmse_yaw = []
        LIDARList = {}
        lidarlist = []
        fail_list = []
        ## get RMSE
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            x_error = x_sensor_localization[idxSensor]
            y_error = y_sensor_localization[idxSensor]
            yaw_error = yaw_sensor_localization[idxSensor]

            if len(x_error) == 0 or len(y_error) == 0 or len(yaw_error) == 0:
                RMSE_x[idxSensor] = 0.0
                rmse_x.append(0.0)
                RMSE_y[idxSensor] = 0.0
                rmse_y.append(0.0)
                RMSE_yaw[idxSensor] = 0.0
                rmse_yaw.append(0.0)
                fail_list.append(idxSensor)

            else:
                for i in range(len(yaw_error)):
                    yaw_error[i] = utils_file.normalized_angle_deg(yaw_error[i])

                x_error_square = np.power(x_error, 2)
                y_error_square = np.power(y_error, 2)
                yaw_error_square = np.power(yaw_error, 2)

                sum_x_error = np.sum(x_error_square)
                sum_y_error = np.sum(y_error_square)
                sum_yaw_error = np.sum(yaw_error_square)

                RMSE_x[idxSensor] = np.sqrt(sum_x_error / len(x_error))
                rmse_x.append(np.sqrt(sum_x_error / len(x_error)))
                RMSE_y[idxSensor] = np.sqrt(sum_y_error / len(y_error))
                rmse_y.append(np.sqrt(sum_y_error / len(y_error)))
                RMSE_yaw[idxSensor] = np.sqrt(sum_yaw_error / len(yaw_error))
                rmse_yaw.append(np.sqrt(sum_yaw_error / len(yaw_error)))

            LIDARList[idxSensor] = 'LiDAR_' + str(idxSensor)
            lidarlist.append('LiDAR_' + str(idxSensor))

        self.df_info = copy.deepcopy(df_info)
        self.Map = copy.deepcopy(Map)
        self.PARM_LIDAR = copy.deepcopy(PARM_LIDAR)
        self.rmse_x = copy.deepcopy(rmse_x)
        self.rmse_y = copy.deepcopy(rmse_y)
        self.rmse_yaw = copy.deepcopy(rmse_yaw)
        self.lidarlist = copy.deepcopy(lidarlist)
        self.fail_list = fail_list
        thread.mutex.unlock()

    def AutoLabel(self, rects, ax):
        """Attach a text label above each bar in *rects*, displaying its height."""
        for rect in rects:
            height = round(rect.get_height(), 4)
            ax.annotate('{}'.format(height),
                        xy=(rect.get_x() + rect.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        rotation=90,
                        size=8,
                        textcoords="offset points",
                        ha='center', va='bottom')

    def GetMaxBarHeight(self, rects):
        max_height = 0.0

        for rect in rects:
            height = rect.get_height()
            if height > max_height:
                max_height = height

        return max_height
