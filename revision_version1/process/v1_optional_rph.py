# -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
@date: 2018-07-12
@version: 0.0.1
"""

# Basic modules in Anaconda
import pandas as pd
import random
from scipy.optimize import minimize
import configparser
import math
import numpy as np
import scipy.optimize


# User defined modules
from process import utils_file
from process import utils_pose

from sklearn import linear_model

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Additional modules
from tqdm import tqdm

# Additional modules
from tqdm import tqdm

# User defined modules
from process import utils_icp
import copy

class RPH:
    def __init__(self, config, importing):
        self.config = config
        self.importing = importing
        self.complete_calibration = False

        # Path and file
        self.export_path = ''
        self.result_calibration_config_file = ''

        # Parameter
        self.EstimateResult = {}

    def fitPLaneLTSQ(self, XYZ):
        # Fits a plane to a point cloud,
        # Where Z = aX + bY + c        ----Eqn #1
        # Rearanging Eqn1: aX + bY -Z +c =0
        # Gives normal (a,b,-1)
        # Normal = (a,b,-1)
        [rows, cols] = XYZ.shape
        G = np.ones((rows, 3))
        G[:, 0] = XYZ[:, 0]  # X
        G[:, 1] = XYZ[:, 1]  # Y
        Z = XYZ[:, 2]
        (a, b, c), resid, rank, s = np.linalg.lstsq(G, Z)
        normal = (a, b, -1)
        nn = np.linalg.norm(normal)
        normal = normal / nn
        return normal

    def fitPlaneSVD(self, XYZ):
        [rows, cols] = XYZ.shape
        # Set up constraint equations of the form  AB = 0,
        # where B is a column vector of the plane coefficients
        # in the form b(1)*X + b(2)*Y +b(3)*Z + b(4) = 0.
        p = (np.ones((rows, 1)))
        AB = np.hstack([XYZ, p])
        [u, d, v] = np.linalg.svd(AB, 0)
        B = v[3, :];  # Solution is last column of v.
        nn = np.linalg.norm(B[0:3])
        B = B / nn
        return B[0:3]

    def fitPlaneEigen(self, XYZ):
        # Works, in this case but don't understand!
        average = sum(XYZ) / XYZ.shape[0]
        covariant = np.cov(XYZ - average)
        eigenvalues, eigenvectors = np.linalg.eig(covariant)
        want_max = eigenvectors[:, eigenvalues.argmax()]
        (c, a, b) = want_max[3:6]  # Do not understand! Why 3:6? Why (c,a,b)?
        normal = np.array([a, b, c])
        nn = np.linalg.norm(normal)
        return normal / nn

    def fitPlaneSolve(self, XYZ):
        X = XYZ[:, 0]
        Y = XYZ[:, 1]
        Z = XYZ[:, 2]
        npts = len(X)
        A = np.array([[sum(X * X), sum(X * Y), sum(X)],
                      [sum(X * Y), sum(Y * Y), sum(Y)],
                      [sum(X), sum(Y), npts]])
        B = np.array([[sum(X * Z), sum(Y * Z), sum(Z)]])
        normal = np.linalg.solve(A, B.T)
        nn = np.linalg.norm(normal)
        normal = normal / nn
        return normal.ravel()

    def fitPlaneOptimize(self, XYZ):
        def residiuals(parameter, f, x, y):
            return [(f[i] - model(parameter, x[i], y[i])) for i in range(len(f))]

        def model(parameter, x, y):
            a, b, c = parameter
            return a * x + b * y + c

        X = XYZ[:, 0]
        Y = XYZ[:, 1]
        Z = XYZ[:, 2]
        p0 = [1., 1., 1.]  # initial guess
        result = scipy.optimize.leastsq(residiuals, p0, args=(Z, X, Y))[0]
        normal = result[0:3]
        nn = np.linalg.norm(normal)
        normal = normal / nn
        return normal

    def find_plane(self, pcd):
        # xyz = np.asarray(pcd.points)
        xyz = np.asarray(pcd)

        XY = xyz[:, :2]

        Z = xyz[:, 2]
        ransac = linear_model.RANSACRegressor(residual_threshold=0.0001)

        ransac.fit(XY, Z)
        a, b = ransac.estimator_.coef_  # 係数
        d = ransac.estimator_.intercept_  # 切片

        return a, b, d  # Z = aX + bY + d

    def angle_rotate(self, a, b, d):
        x = np.arange(30)
        y = np.arange(30)
        X, Y = np.meshgrid(x, y)
        Z = a * X + b * Y + d
        rad = math.atan2(Y[1][0] - Y[0][0], (Z[1][0] - Z[0][0]))
        return rad - math.pi

    def show_graph(self, X, Y, Z):
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot_surface(X, Y, Z)

        plt.show()

    ##############################################################################################################################
    # %% 3. Handeye
    ##############################################################################################################################
    def Calibration(self, thread, args):
        thread.mutex.lock()
        PARM_LIDAR = copy.deepcopy(args[0])
        using_gnss_motion = args[1]
        df_info = self.importing.df_info

        p1 = {}
        p2 = {}
        p3 = {}
        v1 = {}
        v2 = {}
        roll = {}
        pitch = {}
        roll_deg = {}
        pitch_deg = {}
        z = {}
        theta = {}
        theta_deg = {}
        v1_norm = {}
        v2_norm = {}
        theta_2d = {}
        theta_2d_deg = {}
        z_norm = {}
        SVD = {}
        LTSQ = {}
        ransac = {}
        roll_svd = {}
        pitch_svd = {}
        roll_svd_deg = {}
        pitch_svd_deg = {}
        roll_ltsq = {}
        pitch_ltsq = {}
        roll_ltsq_deg = {}
        pitch_ltsq_deg = {}
        roll_ransac = {}
        pitch_ransac = {}
        roll_ransac_deg = {}
        pitch_ransac_deg = {}
        roll_ground_throuth = {}
        pitch_ground_throuth = {}
        roll_svd_error = {}
        pitch_svd_error = {}
        roll_ground_throuth[0] = 10
        roll_ground_throuth[1] = 20
        roll_ground_throuth[2] = 30
        roll_ground_throuth[3] = 10
        pitch_ground_throuth[0] = 10
        pitch_ground_throuth[1] = 20
        pitch_ground_throuth[2] = 30
        pitch_ground_throuth[3] = 20
        LIDARList = {}
        lidarlist = []

        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            LIDARList[idxSensor] = 'LiDAR_' + str(idxSensor)
            lidarlist.append('LiDAR_' + str(idxSensor))

            threshold = 0.7
            x_threshold = 2
            y_threshold = 1.0
            z_threshold = 0.5
            diff_point_xyzdh = []
            diff_gnss_xyzdh = []

            # Remove rows by other sensors
            strColIndex = 'PointCloud_' + str(idxSensor)

            if not using_gnss_motion:
                df_one_info = df_info[['east_m', 'north_m', 'heading', strColIndex]]
                df_one_info = df_one_info.drop(df_info[(df_one_info[strColIndex].values == 0)].index)
            elif using_gnss_motion:
                df_one_info = df_info[['dr_east_m', 'dr_north_m', 'dr_heading', strColIndex]]
                df_one_info.rename(columns={"dr_east_m" : "east_m", "dr_north_m" : "north_m", "dr_heading" : "heading"}, inplace=True)
                df_one_info = df_one_info.drop(df_info[(df_one_info[strColIndex].values == 0)].index)
            # df_one_info = df_info[['east_m', 'north_m', 'heading', strColIndex]]
            # df_one_info = df_one_info.drop(df_info[(df_one_info[strColIndex].values == 0)].index)

            pointcloud1 = self.importing.PointCloudSensorList[idxSensor][int(df_one_info[strColIndex].values[0])]

            '''
            tx = df_one_info['east_m'].values[0]
            ty = df_one_info['north_m'].values[0]
            heading_deg = df_one_info['heading'].values[0]
            heading_rad = heading_deg*np.pi/180

            Transformation_world_to_veh = np.asarray([[np.cos(heading_rad), -np.sin(heading_rad), tx],[np.sin(heading_rad), np.cos(heading_rad), ty],[0., 0., 1.]])
            Transformation_veh_to_world = np.linalg.inv(Transformation_world_to_veh)

            # pointcloud1 = np.hstack([pointcloud1, np.ones((len(pointcloud1),1))])
            pointcloud1_veh = []
            for i in range(len(pointcloud1)):
                pointcloud1_veh.append(Transformation_veh_to_world*pointcloud1[i])

            pointcloud1 = np.asarray(pointcloud1_veh)

            '''
            if pointcloud1.shape[0] < 1:
                continue

            cond = (-y_threshold < pointcloud1[:, 1]) & (pointcloud1[:, 1] < y_threshold) & (
                        -x_threshold < pointcloud1[:, 0]) & (pointcloud1[:, 0] < x_threshold) & (
                               -z_threshold + 7 < pointcloud1[:, 2]) & (pointcloud1[:, 2] < z_threshold + 7)
            # cond = np.sqrt(np.power((pointcloud1[:,0]),2)+np.power((pointcloud1[:,1]),2)) < threshold

            pointcloud2 = {}
            pointcloud2 = pointcloud1[cond]

            p1[idxSensor] = pointcloud2[0, :]
            p2[idxSensor] = pointcloud2[1, :]
            p3[idxSensor] = pointcloud2[2, :]

            v1[idxSensor] = p2[idxSensor] - p1[idxSensor]
            v2[idxSensor] = p3[idxSensor] - p1[idxSensor]

            x1 = v1[idxSensor][0]
            x2 = v2[idxSensor][0]

            y1 = v1[idxSensor][1]
            y2 = v2[idxSensor][1]

            z1 = v1[idxSensor][2]
            z2 = v2[idxSensor][2]

            theta[idxSensor] = np.arccos((x1 * x2 + y1 * y2 + z1 * z2) / (
                        np.sqrt(np.power(x1, 2) + np.power(y1, 2) + np.power(z1, 2)) * np.sqrt(
                    np.power(x2, 2) + np.power(y2, 2) + np.power(z2, 2))))
            theta_deg[idxSensor] = theta[idxSensor] * 180 / np.pi

            z[idxSensor] = np.cross(v1[idxSensor], v2[idxSensor])
            z_norm[idxSensor] = np.sqrt(
                z[idxSensor][0] * z[idxSensor][0] + z[idxSensor][1] * z[idxSensor][1] + z[idxSensor][2] * z[idxSensor][
                    2])
            z[idxSensor] = z[idxSensor] / np.linalg.norm(z[idxSensor])

            v1_norm[idxSensor] = np.sqrt(x1 * x1 + y1 * y1 + z1 * z1)
            v2_norm[idxSensor] = np.sqrt(x2 * x2 + y2 * y2 + z2 * z2)

            theta_2d[idxSensor] = np.arcsin(z_norm[idxSensor] / (v1_norm[idxSensor] * v2_norm[idxSensor]))
            theta_2d_deg[idxSensor] = theta_2d[idxSensor] * 180 / np.pi

            roll[idxSensor] = math.atan2(-z[idxSensor][1], z[idxSensor][2])
            pitch[idxSensor] = math.asin(z[idxSensor][0])

            roll_deg[idxSensor] = roll[idxSensor] * 180 / np.pi
            pitch_deg[idxSensor] = pitch[idxSensor] * 180 / np.pi

            SVD[idxSensor] = self.fitPLaneLTSQ(pointcloud2)
            LTSQ[idxSensor] = self.fitPLaneLTSQ(pointcloud2)

            if SVD[idxSensor][2] < 0:
                SVD[idxSensor] = -SVD[idxSensor]
            if LTSQ[idxSensor][2] < 0:
                LTSQ[idxSensor] = -LTSQ[idxSensor]

            roll_svd[idxSensor] = math.atan2(-SVD[idxSensor][1], SVD[idxSensor][2])
            pitch_svd[idxSensor] = math.asin(SVD[idxSensor][0])
            roll_ltsq[idxSensor] = math.atan2(-LTSQ[idxSensor][1], LTSQ[idxSensor][2])
            pitch_ltsq[idxSensor] = math.asin(LTSQ[idxSensor][0])

            roll_svd_deg[idxSensor] = roll_svd[idxSensor] * 180 / np.pi
            pitch_svd_deg[idxSensor] = pitch_svd[idxSensor] * 180 / np.pi
            roll_ltsq_deg[idxSensor] = roll_ltsq[idxSensor] * 180 / np.pi
            pitch_ltsq_deg[idxSensor] = pitch_ltsq[idxSensor] * 180 / np.pi

            array = np.asarray(self.find_plane(pointcloud2))
            array[2] = -1
            ransac[idxSensor] = array
            ransac[idxSensor] = -1 * ransac[idxSensor]

            roll_ransac[idxSensor] = math.atan2(-ransac[idxSensor][1], ransac[idxSensor][2])
            pitch_ransac[idxSensor] = math.asin(ransac[idxSensor][0])

            roll_ransac_deg[idxSensor] = roll_ransac[idxSensor] * 180 / np.pi
            pitch_ransac_deg[idxSensor] = pitch_ransac[idxSensor] * 180 / np.pi

            roll_svd_error[idxSensor] = np.abs(roll_svd_deg[idxSensor] + roll_ground_throuth[idxSensor])
            pitch_svd_error[idxSensor] = np.abs(roll_svd_deg[idxSensor] + pitch_ground_throuth[idxSensor])

            thread.emit_string.emit('Complete estimate lidar {} calibration'.format(idxSensor))

            estimate = []
            estimate.append(roll_deg[idxSensor])
            estimate.append(pitch_deg[idxSensor])
            # estimate.append(height_deg)
            self.EstimateResult[idxSensor] = estimate

        self.PARM_LIDAR = copy.deepcopy(PARM_LIDAR)
        thread.mutex.unlock()
        print('Complete Estimating RPH')