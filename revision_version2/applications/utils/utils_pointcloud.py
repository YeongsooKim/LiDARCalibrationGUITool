# -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
"""

#import g2o
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors

def cvt_pointcloud_sensor_veh(pointcloud, calib):
    """
    Input: 
        pointcloud: n*3 
        calib: calibration parameter x, y, theta(rad)
        
    output:
        pointcloud in vehicle coordinate
    """
    # Rotation
    rotation_sensor2veh = np.array([
        [np.cos(calib[2]), -np.sin(calib[2]), 0],
        [np.sin(calib[2]), np.cos(calib[2]), 0],
        [0, 0, 1]])

    # translation
    translation_sensor2veh = np.transpose([calib[0],calib[1], 0])

    # Transform sensor coordinate to vehicle coordinate
    pointcloud_veh = np.transpose(np.matmul(rotation_sensor2veh, pointcloud.T)) + translation_sensor2veh
    
    return pointcloud_veh
    
def cvt_pointcloud_sensor_enu(pointcloud, calib, enh):
    """
    Input: 
        pointcloud: n*3 
        calib: calibration parameter x, y, theta(rad)
        enh: pose of east, north, heading(rad)
        
    output:
        pointcloud in enu coordinate
    """
    # Rotation
    rotation_sensor2veh = np.array([
        [np.cos(calib[2]), -np.sin(calib[2]), 0],
        [np.sin(calib[2]), np.cos(calib[2]), 0],
        [0, 0, 1]])

    # translation
    translation_sensor2veh = np.transpose([calib[0],calib[1], 0])

    # Transform sensor coordinate to vehicle coordinate
    pointcloud_veh = np.transpose(np.matmul(rotation_sensor2veh, pointcloud.T)) + translation_sensor2veh

    # Rotation of vehicle
    rotation_veh2enu = np.array([
        [np.cos(enh[2] + np.pi / 2.), -np.sin(enh[2] + np.pi / 2.), 0],
        [np.sin(enh[2] + np.pi / 2.), np.cos(enh[2] + np.pi / 2.), 0],
        [0, 0, 1]])

    # Translation of vehicle
    translation_veh2enu = np.array([enh[0],enh[1], 0])

    # Transform vehicle coordiante to ENU coordinate
    pointcloud_enu = np.transpose(np.matmul(rotation_veh2enu, pointcloud_veh.T)) + translation_veh2enu
    return pointcloud_enu

def cvt_pointcloud_6dof_sensor_enu(pointcloud, calib, enh):
    """
    Input: 
        pointcloud: n*3 
        calib: calibration parameter r, p, y, x, y, z (rad)
        enh: pose of east, north, heading(rad)
        
    output:
        pointcloud in enu coordinate
    """
    # Get parameter
    roll = calib[0]
    pitch = calib[1]
    yaw = calib[2]
    lon = calib[3]
    lat = calib[4]
    hgt = calib[5]
    
    # roll
    rotation_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]])
    
    # pitch
    rotation_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]])
    
    
    # yaw
    rotation_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]])
    
    # Rotation
    rotation_sensor2veh = np.matmul(np.matmul(rotation_yaw, rotation_pitch), rotation_roll)

    # translation
    translation_sensor2veh = np.transpose([lon, lat, hgt])

    # Transform sensor coordinate to vehicle coordinate
    pointcloud_veh = np.transpose(np.matmul(rotation_sensor2veh, pointcloud.T)) + translation_sensor2veh

    # Rotation of vehicle
    rotation_veh2enu = np.array([
        [np.cos(enh[2] + np.pi / 2.), -np.sin(enh[2] + np.pi / 2.), 0],
        [np.sin(enh[2] + np.pi / 2.), np.cos(enh[2] + np.pi / 2.), 0],
        [0, 0, 1]])

    # Translation of vehicle
    translation_veh2enu = np.array([enh[0],enh[1], 0])

    # Transform vehicle coordiante to ENU coordinate
    pointcloud_enu = np.transpose(np.matmul(rotation_veh2enu, pointcloud_veh.T)) + translation_veh2enu
    return pointcloud_enu

def plane_fitting(pointcloud, debug = False):
    # Ax + By + Cz + D = 0
    tmp_A = np.concatenate((pointcloud[:,0:2],np.full((pointcloud.shape[0], 1), 1)), axis=1)
    A = np.matrix(tmp_A)
    b = np.matrix(pointcloud[:,2]).T
    fit = (A.T * A).I * A.T * b
    
    # Parameter
    A = float(fit[0])
    B = float(fit[1])
    C = float(-1)
    D = float(fit[2])
    
    # When debug is True, plot plane and pointcloud s
    if debug == True:
        plt.figure()
        ax = plt.subplot(111, projection='3d')
        ax.scatter(pointcloud[:,0], pointcloud[:,1], pointcloud[:,2], color='b')        
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        X,Y = np.meshgrid(np.arange(xlim[0], xlim[1], (xlim[1]-xlim[0])/10),
                          np.arange(ylim[0], ylim[1], (xlim[1]-xlim[0])/10))
        Z = np.zeros(X.shape)
        for r in range(X.shape[0]):
            for c in range(X.shape[1]):
                Z[r,c] = A * X[r,c] + B * Y[r,c] + D
        ax.plot_wireframe(X,Y,Z, color='k')
        
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        plt.show()

    return A,B,C,D

def get_distance_from_plane(point, plane_param):
    # Ax+By+Cz+D=0
    err = np.abs(point[0]*plane_param[0]+point[1]*plane_param[1]+point[2]*plane_param[2]+plane_param[3])/np.sqrt(plane_param[0]**2 + plane_param[1]**2 + plane_param[2]**2)
    return err

def filter_pointcloud(pointcloud,min_x,max_x,min_y,max_y,min_z,max_z):
    remove_filter = pointcloud[:,0:1] < float(min_x)
    remove_filter = np.logical_or(remove_filter, pointcloud[:,0:1] > float(max_x))
    remove_filter = np.logical_or(remove_filter, pointcloud[:,1:2] < float(min_y))
    remove_filter = np.logical_or(remove_filter, pointcloud[:,1:2] > float(max_y))
    remove_filter = np.logical_or(remove_filter, pointcloud[:,2:3] < float(min_z))
    remove_filter = np.logical_or(remove_filter, pointcloud[:,2:3] > float(max_z))

    # filtering
    filtered_pointcloud = np.ma.compress(np.bitwise_not(np.ravel(remove_filter)),pointcloud,axis=0)
    return filtered_pointcloud

def get_rotation_matrix(roll, pitch, yaw):
    yawMatrix = np.matrix([
    [np.cos(yaw), -np.sin(yaw), 0, 0],
    [np.sin(yaw), np.cos(yaw), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]])
    
    pitchMatrix = np.matrix([
    [np.cos(pitch), 0, np.sin(pitch), 0],
    [0, 1, 0, 0],
    [-np.sin(pitch), 0, np.cos(pitch), 0],
    [0, 0, 0, 1]])
    
    rollMatrix = np.matrix([
    [1, 0, 0, 0],
    [0, np.cos(roll), -np.sin(roll), 0],
    [0, np.sin(roll), np.cos(roll), 0],
    [0, 0, 0, 1]])
    
    RotationMatrix = yawMatrix * pitchMatrix * rollMatrix
    return RotationMatrix

def get_translation_matrix(x, y, z):   
    TranslationMatrix = np.matrix([
    [1, 0, 0, x],
    [0, 1, 0, y],
    [0, 0, 1, z],
    [0, 0, 0, 1]])
    return TranslationMatrix

def apply_calib_param(pointcloud, calib):  
    # Rotation
    RotationMatrix = get_rotation_matrix(calib[0],calib[1],calib[2])

    # Translation
    TranslationMatrix = get_translation_matrix(calib[3],calib[4],calib[5])

    # Transform
    TransformMatrix = TranslationMatrix * RotationMatrix
    return_pointcloud = transform_pointcloud(pointcloud, TransformMatrix)
    
    return return_pointcloud

def transform_pointcloud(pointcloud, transform):
    return_pointcloud = np.copy(pointcloud)
    
    # Get point cloud
    pointcloud4 = np.hstack([return_pointcloud[:,0:3], np.ones((return_pointcloud.shape[0], 1))])
    
    # Apply pointcloud
    pointcloud_calib = np.transpose(np.matrix(transform) * pointcloud4.T)

    # Replace the point cloud data
    return_pointcloud[:,0:3] = pointcloud_calib[:,0:3]
    return return_pointcloud
    
#def match_gicp(pres_pc, next_pc, nn_min_dist = 1., num_optim = 5, num_matching = 5):
#    translate_pose = np.array([0,0,0])
#    for idxMatch in list(range(num_matching)):
#        print(idxMatch)
#        print(translate_pose)
#            
#        optimizer = g2o.SparseOptimizer()
#        solver = g2o.BlockSolverX(g2o.LinearSolverDenseX())
#        algorithm = g2o.OptimizationAlgorithmLevenberg(solver)
#        optimizer.set_algorithm(algorithm)
#        
#        # Present pose
#        pres_trans = np.array([0, 0, 0])
#        pres_pose = g2o.Isometry3d(np.identity(3), pres_trans)
#        pres_vc = g2o.VertexSE3()
#        pres_vc.set_id(0)
#        pres_vc.set_estimate(pres_pose)
#        pres_vc.set_fixed(True)
#        optimizer.add_vertex(pres_vc)
#        
#        # Next pose
#        next_trans = translate_pose
#        next_pose = g2o.Isometry3d(np.identity(3), next_trans)
#        next_vc = g2o.VertexSE3()
#        next_vc.set_id(1)
#        next_vc.set_estimate(next_pose)
#        optimizer.add_vertex(next_vc)
#        
#        # Generate NN using present point cloud
#        nn = NearestNeighbors(n_neighbors=1)
#        nn.fit(pres_pc[:,0:3])
#        
#        # Find neasrest neighbors
#        closest_distances, closest_indices = nn.kneighbors(next_pc[:,0:3], return_distance=True)
#    
#        plt.figure()
#        plt.plot(pres_pc[:,0],pres_pc[:,1],'.',label='Pres')
#        plt.plot(next_pc[:,0],next_pc[:,1],'.',label='Next')
#        
#        # Matching
#        for i in list(range(closest_indices.shape[0])):
#            idx_pres_pc = int(closest_indices[i])
#            idx_next_pc = i
#            
#            distance_matching = float(closest_distances[i])
#            
#            # Check matching distance 
#            if distance_matching > nn_min_dist:
#                continue
#            
#            # Get associated points
#            pres_point = pres_pc[idx_pres_pc]
#            next_point = next_pc[idx_next_pc]
#            
#            # Measure
#            meas = g2o.EdgeGICP()
#            meas.pos0 = pres_point
#            meas.pos1 = next_point
#    
#            edge = g2o.Edge_V_V_GICP()
#            edge.set_vertex(0, optimizer.vertex(0))
#            edge.set_vertex(1, optimizer.vertex(1))
#            edge.set_measurement(meas)
#            edge.set_information(meas.prec0(0.01))
#    
#            optimizer.add_edge(edge)
#            
#            plt.plot([pres_point[0],next_point[0]],[pres_point[1],next_point[1]],'-',label='Match')
#        
#        # Compute the relation
#        optimizer.initialize_optimization()
#        optimizer.compute_active_errors()
#        
#        optimizer.set_verbose(True)
#        optimizer.optimize(num_optim)
#        
#        trans = optimizer.vertex(1).estimate().t
#        rot = optimizer.vertex(1).estimate().R
#        
#        transform = np.eye(4)
#        transform[0:3,0:3] = rot 
#        transform[0,3] = trans[0]
#        transform[1,3] = trans[1]
#        transform[2,3] = trans[2]
#    
#        translate_pose = trans
#    return transform