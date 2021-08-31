# -*- coding: utf-8 -*-

import numpy as np
from applications.utils import utils_pointcloud
from sklearn.neighbors import NearestNeighbors

global strFile
global nFeval

def compute_single_err(CalibH_rad, CalibX_m, CalibY_m, pose, pointcloud, PARM_IM, PARM_SO, thread):
    ##################
    #Global variables
    global strFile
    global nFeval

    nFeval += 1

    ##################
    # Calib param
    CalibH_d = CalibH_rad[0] * 180. / np.pi

    # Get calibration data
    calib_param = []
    calib_param.append(CalibX_m)
    calib_param.append(CalibY_m)
    calib_param.append(CalibH_rad[0])
    
    ##################
    # Get pose
    east = pose[0]
    # north = pose[1]
    # heading = pose[2]
    index_pointcloud = pose[3]

    ##################
    # Sampling the pose bsaed on pose sampling interval
    num_pose = len(east)
    interval = PARM_IM['SamplingInterval'] * 8
    if interval < 1:
        interval = 1
    idx_sampling_pose = list(range(0, num_pose, interval))

    ##################
    # Accumulation of point cloud
    accum_point_enup = np.empty((0,4))
    for idx_pose in idx_sampling_pose:
        # Convert raw to enu
        point_sensor = pointcloud[int(index_pointcloud[idx_pose])][:,0:3]
        point_enu = utils_pointcloud.cvt_pointcloud_sensor_enu(point_sensor, calib_param, pose[0:3, idx_pose])

        # Add index
        point_enup = np.concatenate((point_enu,np.full((point_enu.shape[0], 1), idx_pose)), axis=1)

        # Accumulate the point cloud
        accum_point_enup = np.vstack([accum_point_enup, point_enup])

    ##################
    # pose based matching
    pose_err = []
    for idx_pose in idx_sampling_pose:
         # Split the selected and excepted point cloud
         sel_point = accum_point_enup[accum_point_enup[:,3] == idx_pose,0:3]
         exc_point = accum_point_enup[accum_point_enup[:,3] != idx_pose,0:3]

         # Generate nearest neighbors
         nearest_neighbor_plane = NearestNeighbors(n_neighbors=PARM_SO['NumPointsPlaneModeling'])
         nearest_neighbor_plane.fit(sel_point)

         # Sampling excepted point cloud
         num_point = exc_point.shape[0]
         interval = int(1./PARM_SO['PointSamplingRatio'])
         if interval < 1:
             interval = 1
         idx_sampling_point = list(range(0, num_point, interval))

         # Set search point
         project_points = []
         for idx_point in idx_sampling_point:
             project_points .append(exc_point[idx_point])
         project_points = np.array(project_points)

         # Seach matched points
         closest_distances, closest_indices = nearest_neighbor_plane.kneighbors(project_points, return_distance=True)

         # get plane fitting
         point_err = []
         for i in list(range(closest_indices.shape[0])):
             plane_point = []
             for j in list(range(closest_indices[i].shape[0])):
                 if closest_distances[i][j] < PARM_SO['OutlierDistance_m']:
                     plane_point.append(sel_point[closest_indices[i][j]])
             plane_point = np.array(plane_point)
             if plane_point.shape[0] < 4:
                 continue
             plane_param = utils_pointcloud.plane_fitting(plane_point)
             if (plane_param[0] == 0) & (plane_param[1] == 0) & (plane_param[2] == 0) & (plane_param[3] == 0):
                 continue
             else:
                 # Get error
                 err = utils_pointcloud.get_distance_from_plane(project_points[i], plane_param)
             point_err.append(abs(err))

         # Condition that the number of point err is low
         if len(point_err) < 1:
             continue

         mean_point_err = np.mean(np.array(point_err))
         pose_err.append(mean_point_err)

         print("progress {} %".format(idx_pose/num_pose * 100))

    # Condition that the number of pose err is low
    if len(pose_err) < 1:
        err = compute_single_err(CalibH_rad[0], CalibX_m, CalibY_m, pose, pointcloud, PARM_IM, PARM_SO, thread)
    else:
        err = np.mean(np.array(pose_err))

    thread.emit_string.emit('{0:>10}    {1:4d} {2:>15}   {3:>15}'.format(strFile, nFeval, str(round(CalibH_d,8)), str(round(err,8))))

    # if not thread._status:
    #     is_break = True
    # #     thread.cond.wait(thread.mutex)

    print('{0:}   {1:4d}   {2:3.6f}  {3:3.6f}'.format(strFile, nFeval, CalibH_d, err))

    # return error
    return err

def compute_multi_err(CalibH_rad, CalibX_m, CalibY_m, pose, pointcloud, accum_point_ref, nearest_neighbor, PARM_IM, PARM_MO, thread):
    ##################
    #Global variables
    global strFile
    global nFeval
    
    nFeval += 1

    ##################
    # Calib param
    CalibH_d = CalibH_rad[0] * 180. / np.pi

    # Get calibration data
    calib_param = []
    calib_param.append(CalibX_m)
    calib_param.append(CalibY_m)
    calib_param.append(CalibH_rad[0])
    
    ##################
    # Get pose
    east = pose[0]
    north = pose[1]
    heading = pose[2]
    index_pointcloud = pose[3]

    ##################
    # Sampling the pose bsaed on pose sampling interval
    num_pose = len(east)
    interval = PARM_IM['SamplingInterval'] * 10
    if interval < 1:
        interval = 1
    idx_sampling_pose = list(range(0, num_pose, interval))

    ##################
    # Accumulation of point cloud
    accum_point_enup = np.empty((0,4))
    for idx_pose in idx_sampling_pose:
        # Convert raw to enu
        point_sensor = pointcloud[int(index_pointcloud[idx_pose])][:,0:3]
        point_enu = utils_pointcloud.cvt_pointcloud_sensor_enu(point_sensor, calib_param, pose[0:3, idx_pose])

        # Add index
        point_enup = np.concatenate((point_enu,np.full((point_enu.shape[0], 1), idx_pose)), axis=1)

        # Accumulate the point cloud
        accum_point_enup = np.vstack([accum_point_enup, point_enup])

    ##################
    # pose based matching
    # Sampling excepted point cloud
    num_point = accum_point_enup.shape[0]
    interval = int(1./PARM_MO['PointSamplingRatio'])
    if interval < 1:
        interval = 1
    idx_sampling_point = list(range(0, num_point, interval))

    # Set search point
    project_points = []
    for idx_point in idx_sampling_point:
        project_points.append(accum_point_enup[idx_point])
    project_points = np.array(project_points)

    # Seach matched points
    closest_distances, closest_indices = nearest_neighbor.kneighbors(project_points, return_distance=True)

    # get plane fitting
    point_err = []
    for i in list(range(closest_indices.shape[0])):
        plane_point = []
        for j in list(range(closest_indices[i].shape[0])):
            if closest_distances[i][j] < PARM_MO['OutlierDistance_m']:
                plane_point.append(accum_point_ref[closest_indices[i][j]])
        plane_point = np.array(plane_point)
        if plane_point.shape[0] < 4:
            continue
        plane_param = utils_pointcloud.plane_fitting(plane_point)

        # Get error
        err = utils_pointcloud.get_distance_from_plane(project_points[i], plane_param)
        point_err.append(abs(err))

    # Compute mean
    err = np.mean(np.array(point_err))
    # result_str = '{0:}   {1:4d}   {2:3.6f}   {3:3.6f}'.format(strFile, nFeval, CalibH_d, err)
    # print

    thread.emit_string.emit('{0:>10}    {1:4d} {2:>15}   {3:>15}'.format(strFile, nFeval, str(round(CalibH_d,8)), str(round(err,8))))

    # if not thread._status:
    #     is_break = True
    # #     thread.cond.wait(thread.mutex)

    print('{0:}   {1:4d}   {2:3.6f}   {3:3.6f}'.format(strFile, nFeval, CalibH_d, err))

    return err
    