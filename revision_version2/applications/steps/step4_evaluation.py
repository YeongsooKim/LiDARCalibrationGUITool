 # -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
@date: 2018-07-12
@version: 0.0.1
"""

import numpy as np
import copy
import open3d as o3d

from tqdm import tqdm
from applications.utils import utils_icp, utils_file


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
        print(zrp_calib)


        # Get calibration data
        tmp_df_info = copy.deepcopy(self.importing.df_info)

        # Limit time
        df_info = tmp_df_info.drop(
            tmp_df_info[(tmp_df_info.index < start_time) | (tmp_df_info.index > end_time)].index)

        init_T = np.array([[np.cos(0 * np.pi / 180), -np.sin(0 * np.pi / 180), 0., 0.],
                           [np.sin(0 * np.pi / 180), np.cos(0 * np.pi / 180), 0., 0.],
                           [0., 0., 1., 0.],
                           [0., 0., 0., 1.]])

        Map = {}
        pred_arr = {}  # optimization의 prediction로 사용할 list 생성
        ref_arr = {}  # optimization의 reference로 사용할 list 생성
        opt_arr = {}  # optimization list 생성

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
            ref_each_localization = []
            pred_each_localization = []
            error_each_localization = []
            x_each_localization = []
            y_each_localization = []
            yaw_each_localization = []
            threshold = 30

            calib = CalibrationParam[idxSensor]
            T_calib = np.array([[np.cos(calib[2]), -np.sin(calib[2]), calib[3]],
                                [np.sin(calib[2]), np.cos(calib[2]), calib[4]],
                                [0, 0, 1]])

            # Remove rows by other sensors
            strColIndex = 'XYZRGB_' + str(idxSensor)  # PointCloud_n 이름 생성
            LiDAR_list[idxSensor] = strColIndex

            if not self.importing.has_gnss_file:
                df_info['east_m'] = df_info['dr_east_m']
                df_info['north_m'] = df_info['dr_north_m']
                df_info['heading'] = df_info['dr_heading']
            if not self.importing.has_motion_file:
                df_info['dr_east_m'] = df_info['east_m']
                df_info['dr_north_m'] = df_info['north_m']
                df_info['dr_heading'] = df_info['heading']

            df_one_info = df_info[['east_m', 'north_m', 'heading', 'dr_east_m', 'dr_north_m', 'dr_heading', strColIndex]]  # df_info의 'east','north','heading',...,XYZRGB 하나씩 분리해서 저장
            df_one_info = df_one_info.drop(df_info[(df_one_info[
                                                        strColIndex].values == 0)].index)  # df_one_info에서 strColIndex(PointCloud_n)의 value가 0인 값들 다 제외 --> 값이 있는 애들만 남겨놓음

            # Sampling based on interval
            df_sampled_info = df_one_info.iloc[::self.config.PARM_EV['SamplingInterval'],
                              :]  # Sampling 간격만큼 분할(추출)함 ex) samplinginterval = 2이면 2행마다 하나 추출
            df_sampled_info.heading = df_sampled_info.heading + 90

            l = range(len(df_sampled_info))  # df_sampled_info의 길이만큼 range생성 후 list로 변환
            # l = [0,1,2,3,...,len(df_sampled_info)-1]

            ##-----------------------------------------------------------------------------------------------------------------------------
            # 6-1. Generating a "HD map" global point cloud map from RTK-GNSS and init calibration
            ##-----------------------------------------------------------------------------------------------------------------------------
            HD_map = np.empty((0, 3))
            # T_calib = np.delete(init_T, (2), axis=0)
            # T_calib = np.delete(T_calib, (2), axis=1)

            ### Make HD Map

            # pbar2 = tqdm(l)

            print('\n----------- Start Mapping -----------')
            # HD Map 생성
            pbar = tqdm(l)  # idx_pair progress bar 생성
            iteration_size = len(l)
            thread.emit_string.emit('Map generating LiDAR {}'.format(idxSensor))
            index = 0
            for j in pbar:  # make hd map

                iteration_ratio = float(index + 1) / float(iteration_size)
                iteration_percentage = iteration_ratio * 100

                epoch_ratio = (iteration_ratio + p_index) / (float(lidar_len) * 10) # job cost of mapping is one and cost of evauation is nine
                epoch_percentage = epoch_ratio * 100

                thread.change_value.emit(int(epoch_percentage))

                pbar.set_description("Progress of Mapping")  # 상태바 naming
                # pbar2.set_description("Progress of HD Map")   # 상태바 naming

                # Get point clouds
                map_pointcloud = {}

                map_pointcloud = self.importing.PointCloudSensorList[idxSensor][int(df_sampled_info[strColIndex].values[j])]  # df_sampled_info의 i번째 값(PointCloud index)에 맞는 pointcloud 데이터 PointCloudSensorList에서 불러옴
                map_pointcloud[:, 2] = 1  # pointcloud의 z값 1로 projection (homogeneous form)

                # if pointcloud.shape[0] < 1:
                #    continue
                # pointcloud = pointcloud[0::2]   # 짝수번째 행만 추출
                # pointcloud = pointcloud[1::2]   # 홀수번째 행만 추출

                # Get GNSS
                map_east = df_sampled_info['east_m'].values[j]  # df_sampled_info의 i번째 'east_m'값 저장
                map_north = df_sampled_info['north_m'].values[j]  # df_sampled_info의 i번째 'north_m'값 저장
                map_yaw = df_sampled_info['heading'].values[j]  # df_sampled_info의 i번째 'heading'값 저장 [deg]
                map_yaw = utils_file.normalized_angle_deg(map_yaw)
                map_yaw_rad = map_yaw * np.pi / 180

                # Get Transformation world to vehicle
                transformation_world_to_veh = np.array([[np.cos(map_yaw_rad), -np.sin(map_yaw_rad), map_east],
                                                        [np.sin(map_yaw_rad), np.cos(map_yaw_rad), map_north],
                                                        [0., 0., 1.]])

                # Get Transformation vehicle to world
                transformation_veh_to_world = np.linalg.inv(transformation_world_to_veh)

                # Get transformation world to lidar
                transformation_world_to_lidar = np.matmul(transformation_world_to_veh, T_calib)

                point = np.matmul(transformation_world_to_lidar, np.transpose(map_pointcloud))

                HD_map = np.vstack([HD_map, np.transpose(point)])

                index = index + 1

            p_index = p_index + 1.0

            Map[idxSensor] = HD_map

            ##-----------------------------------------------------------------------------------------------------------------------------
            # 6-2. Localization
            ##-----------------------------------------------------------------------------------------------------------------------------

            pbar = tqdm(l)  # idx_pair progress bar 생성

            iteration_size = len(l)
            index = 0
            thread.emit_string.emit('Start Evaluation LiDAR_{}'.format(idxSensor))
            for i in pbar:
                # Display progress
                iteration_ratio = float(index + 1) / float(iteration_size)
                iteration_percentage = iteration_ratio * 100

                epoch_ratio = (9 * iteration_ratio + p_index)/(float(lidar_len)*10)
                epoch_percentage = epoch_ratio*100

                thread.change_value.emit(int(epoch_percentage))

                pbar.set_description("Evaluation" + str(idxSensor))  # 상태바 naming

                pointcloud_in_lidar = self.importing.PointCloudSensorList[idxSensor][int(df_sampled_info[strColIndex].values[i])]
                pointcloud_in_lidar[:, 2] = 1

                # pointcloud in veh frame
                pointcloud = np.matmul(T_calib, np.transpose(pointcloud_in_lidar))
                pointcloud = np.transpose(pointcloud)
                # pointcloud[:,2]=1

                # Prediction using DR
                ref_east = df_sampled_info['east_m'].values[i]
                ref_north = df_sampled_info['north_m'].values[i]
                ref_yaw = df_sampled_info['heading'].values[i]
                ref_yaw = utils_file.normalized_angle_deg(ref_yaw)
                ref_yaw_rad = ref_yaw * np.pi / 180

                ref_transformation_world_to_veh = np.array([[np.cos(ref_yaw_rad), -np.sin(ref_yaw_rad), ref_east],
                                                            [np.sin(ref_yaw_rad), np.cos(ref_yaw_rad), ref_north],
                                                            [0., 0., 1.]])

                ref_transformation_veh_to_world = np.linalg.inv(ref_transformation_world_to_veh)

                # Prediction using DR
                pred_east = df_sampled_info['dr_east_m'].values[i]
                pred_north = df_sampled_info['dr_north_m'].values[i]
                pred_yaw = df_sampled_info['dr_heading'].values[i]
                pred_yaw = utils_file.normalized_angle_deg(pred_yaw)
                pred_yaw_rad = ref_yaw * np.pi / 180

                pred_transformation_world_to_veh = np.array([[np.cos(pred_yaw_rad), -np.sin(pred_yaw_rad), pred_east],
                                                             [np.sin(pred_yaw_rad), np.cos(pred_yaw_rad), pred_north],
                                                             [0., 0., 1.]])

                pred_transformation_veh_to_world = np.linalg.inv(pred_transformation_world_to_veh)

                # Update ICP Map Matching
                vehicle_origin = np.array([ref_east, ref_north])
                # pred_east, pred_north기준으로 threshold만큼 ROI 설정
                cond = (ref_east - threshold < HD_map[:, 0]) & (HD_map[:, 0] < ref_east + threshold) & (
                            ref_north - threshold < HD_map[:, 1]) & (HD_map[:, 1] < ref_north + threshold)
                cond = np.sqrt(
                    np.power((HD_map[:, 0] - ref_east), 2) + np.power((HD_map[:, 1] - ref_north), 2)) < threshold

                map_in_ROI = {}
                map_in_ROI = HD_map[cond]

                # Transformation HD Map in World Frame
                # to HD Map in Vehicle Frame
                HDMap_veh = np.matmul(pred_transformation_veh_to_world, np.transpose(map_in_ROI))
                HDMap_veh = np.transpose(HDMap_veh)

                # Downsampling HDMap_veh
                HDMap_veh_pcd = o3d.geometry.PointCloud()
                HDMap_veh_pcd.points = o3d.utility.Vector3dVector(HDMap_veh)
                HDMap_veh_pcd.voxel_down_sample(0.3)

                HDMap_veh = np.asarray(HDMap_veh_pcd.points)

                # Get Transformation between HD Map in Vehicle Frame and Pointcloud in LiDAR Frame
                # Map Matching
                print('\n----------- Start ICP -----------')

                # ICP Registration
                transform_point, distances, iterations, converged = utils_icp.icp_NM(pointcloud[:, 0:3],
                                                                                     HDMap_veh[:, 0:3],
                                                                                     init_pose=init_T,
                                                                                     tolerance=0.0001,
                                                                                     max_iterations=100,
                                                                                     rm_outlier_dist=30)

                print('\n----------- Finish ICP -----------')

                # Transformation between HD Map and pointcloud in veh frame
                transformation_result = transform_point
                # Change size 4by4 to 3by3
                transformation_result = np.delete(transformation_result, (2), axis=0)
                transformation_result = np.delete(transformation_result, (2), axis=1)
                # transformation_veh_to_lidar

                if converged == False:
                    index += 1
                    continue

                # Map matching ransformation by map matching
                transformation_prediction = transformation_result

                # Get the transformation world to vehicle (원래는 ref_transformation_world_to_veh이 아닌, dr 곱해줘양함)
                pred_transformation_world_to_veh = np.matmul(pred_transformation_world_to_veh,
                                                             transformation_prediction)

                pred_east = pred_transformation_world_to_veh[0][2]
                pred_north = pred_transformation_world_to_veh[1][2]
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
        # print("time :", time.time() - start)  # 현재시각 - 시작시간 = 실행 시간

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
        ## get RMSE
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            x_error = x_sensor_localization[idxSensor]
            y_error = y_sensor_localization[idxSensor]
            yaw_error = yaw_sensor_localization[idxSensor]
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
