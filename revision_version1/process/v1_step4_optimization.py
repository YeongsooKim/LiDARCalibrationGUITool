# -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
@date: 2018-07-12
@version: 0.0.1
"""

# Basic modules in Anaconda
import numpy as np
from sklearn.neighbors import NearestNeighbors
from scipy.optimize import minimize

# Additional modules
from tqdm import tqdm

# User defined modules
from process import utils_icp
from process import utils_pointcloud
from process import utils_cost_func

class Optimization:
    def __init__(self, config, importing):
        self.config = config
        self.importing = importing

        # Path and file
        self.config_lidar_file = ''
        self.export_path = ''
        self.result_calibration_config_file = ''

        # Parameter
        self.progress = ''
        self.complete_calibration = False

        self.initial_calibration_param = {}
        self.CalibrationParam = {}

    ##############################################################################################################################
    # %% 3. Optimization
    ##############################################################################################################################
    def Calibration(self, thread, args):
        start_time = args[0]
        end_time = args[1]
        PARM_LIDAR = args[2]
        df_info = self.importing.df_info

        # Limit time
        df_info = df_info.drop(
            df_info[(df_info.index < start_time) | (df_info.index > end_time)].index)
        ##############################################################################################################################
        # %% 3. Multiple optimization
        ##############################################################################################################################

        # -----------------------------------------------------------------------------------------------------------------------------
        # 3-1.  Accumulation
        # -----------------------------------------------------------------------------------------------------------------------------

        ##################
        # Get calibration data
        idxSensor = PARM_LIDAR['PrincipalSensor']
        calib_param = self.initial_calibration_param[idxSensor]
        self.CalibrationParam.clear()
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            self.CalibrationParam[idxSensor] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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
        pointcloud = self.importing.PointCloudSensorList[idxSensor]

        ##################
        # Sampling the pose bsaed on pose sampling interval
        num_pose = pose.shape[1]
        interval = int(1. / self.config.PARM_MO['PoseSamplingRatio'])
        if interval < 1:
            interval = 1
        idx_sampling_pose = list(range(0, num_pose, interval))

        ##################
        # Accumulation of point cloud
        accum_point_enup = np.empty((0, 4))
        for idx_pose in idx_sampling_pose:
            # Convert raw to enu
            point_sensor = pointcloud[int(index_pointcloud[idx_pose])][:, 0:3]
            point_enu = utils_pointcloud.cvt_pointcloud_6dof_sensor_enu(point_sensor, calib_param, pose[0:3, idx_pose])

            # Add index
            point_enup = np.concatenate((point_enu, np.full((point_enu.shape[0], 1), idx_pose)), axis=1)

            # Accumulate the point cloud
            accum_point_enup = np.vstack([accum_point_enup, point_enup])

        ##################
        # Generate nearest neighbors
        nearest_neighbor = NearestNeighbors(n_neighbors=self.config.PARM_MO['NumPointsPlaneModeling'])
        nearest_neighbor.fit(accum_point_enup)

        # -----------------------------------------------------------------------------------------------------------------------------
        # 3-2.  Optimization
        # -----------------------------------------------------------------------------------------------------------------------------

        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            # Exclude the principal sensor
            thread.mutex.lock()
            if idxSensor == PARM_LIDAR['PrincipalSensor']:
                thread.emit_string.emit(str('Complete LiDAR {} calibration'.format(idxSensor)))
                thread.mutex.unlock()
                continue

            # Get calibration data
            calib_param = self.initial_calibration_param[idxSensor]

            # Remove rows by other sensors
            strColIndex = 'PointCloud_' + str(idxSensor)
            df_one_info = df_info[['east_m', 'north_m', 'heading', strColIndex]]
            df_one_info = df_one_info.drop(df_info[(df_one_info[strColIndex].values == 0)].index)

            ##### Arguments
            # Get position
            pose = df_one_info['east_m'].values
            pose = np.vstack([pose, df_one_info['north_m'].values])
            pose = np.vstack([pose, df_one_info['heading'].values * np.pi / 180.])
            pose = np.vstack([pose, df_one_info[strColIndex].values])

            # Get Point cloud list
            pointcloud = self.importing.PointCloudSensorList[idxSensor]

            ##### cost function for optimization
            utils_cost_func.strFile = 'PointCloud_' + str(idxSensor)
            utils_cost_func.nFeval = 0
            res = minimize(utils_cost_func.compute_multi_err,
                           calib_param[2],
                           args=(calib_param[3], calib_param[4], pose, pointcloud, accum_point_enup, nearest_neighbor,
                                 self.config.PARM_MO, thread, self.CalibrationParam, idxSensor),
                           method='Powell',
                           options={'ftol': 1e-10, 'disp': True})
            thread.emit_string.emit(str('Complete LiDAR {} calibration'.format(idxSensor)))
            thread.mutex.unlock()
            # set data
            self.CalibrationParam[idxSensor][2] = float(res.x)


        diff_point_xyzdh_dict = {}
        diff_gnss_xyzdh_dict = {}
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            diff_point_xyzdh = []
            diff_gnss_xyzdh = []

            # Remove rows by other sensors
            strColIndex = 'PointCloud_' + str(idxSensor)
            df_one_info = df_info[['east_m', 'north_m', 'heading', strColIndex]]
            df_one_info = df_one_info.drop(df_info[(df_one_info[strColIndex].values == 0)].index)

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
            index = 0
            for i, j in pbar:

                pbar.set_description("PointCloud_" + str(idxSensor))
                # Get point clouds
                pointcloud1 = self.importing.PointCloudSensorList[idxSensor][int(df_sampled_info[strColIndex].values[i])]
                pointcloud2 = self.importing.PointCloudSensorList[idxSensor][int(df_sampled_info[strColIndex].values[j])]

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

        # -----------------------------------------------------------------------------------------------------------------------------
        # 3-2. Solve the A*X=X*B
        # -----------------------------------------------------------------------------------------------------------------------------

        # Construction of matrix for hand eye calibration
        self.label = []
        self.calib_x = []
        self.calib_y = []
        self.calib_yaw = []
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
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

        print("Complete optimization calibration")
