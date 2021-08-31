# Basic modules in Anaconda
import os
import numpy as np
import matplotlib.pyplot as plt
import random
from sklearn.neighbors import NearestNeighbors
from sklearn import linear_model
import math
import scipy.optimize
import scipy
#import pcl


def fitPLaneLTSQ(XYZ):
    # Fits a plane to a point cloud, 
    # Where Z = aX + bY + c        ----Eqn #1
    # Rearanging Eqn1: aX + bY -Z +c =0
    # Gives normal (a,b,-1)
    # Normal = (a,b,-1)
    [rows,cols] = XYZ.shape
    G = np.ones((rows,3))
    G[:,0] = XYZ[:,0]  #X
    G[:,1] = XYZ[:,1]  #Y
    Z = XYZ[:,2]
    (a,b,c),resid,rank,s = np.linalg.lstsq(G,Z, rcond=-1) 
    normal = (a,b,-1)
    nn = np.linalg.norm(normal)
    normal = normal / nn

    return normal


def fitPlaneSVD(XYZ):
    [rows,cols] = XYZ.shape
    # Set up constraint equations of the form  AB = 0,
    # where B is a column vector of the plane coefficients
    # in the form b(1)*X + b(2)*Y +b(3)*Z + b(4) = 0.
    p = (np.ones((rows,1)))
    AB = np.hstack([XYZ,p])
    [u, d, v] = scipy.linalg.svd(AB,0)
    B = v[3,:]                    # Solution is last column of v.
    if B[0]==0.:
        B[0] = 10e-20

    if B[1]==0.:
        B[1] = 10e-20

    nn = np.linalg.norm(B[0:3])
    B = B / nn
    return B[0:3]


def fitPlaneEigen(XYZ):
    # Works, in this case but don't understand!
    average=sum(XYZ)/XYZ.shape[0]
    covariant=np.cov(XYZ - average)
    eigenvalues,eigenvectors = np.linalg.eig(covariant)
    want_max = eigenvectors[:,eigenvalues.argmax()]
    (c,a,b) = want_max[3:6]    # Do not understand! Why 3:6? Why (c,a,b)?
    normal = np.array([a,b,c])
    nn = np.linalg.norm(normal)
    return normal / nn  

def fitPlaneSolve(XYZ):
    X = XYZ[:,0]
    Y = XYZ[:,1]
    Z = XYZ[:,2] 
    npts = len(X)
    A = np.array([ [sum(X*X), sum(X*Y), sum(X)],
                   [sum(X*Y), sum(Y*Y), sum(Y)],
                   [sum(X),   sum(Y), npts] ])
    B = np.array([ [sum(X*Z), sum(Y*Z), sum(Z)] ])
    normal = np.linalg.solve(A,B.T)
    nn = np.linalg.norm(normal)
    normal = normal / nn
    return normal.ravel()

def fitPlaneOptimize(XYZ):
    def residiuals(parameter,f,x,y):
        return [(f[i] - model(parameter,x[i],y[i])) for i in range(len(f))]


    def model(parameter, x, y):
        a, b, c = parameter
        return a*x + b*y + c

    X = XYZ[:,0]
    Y = XYZ[:,1]
    Z = XYZ[:,2]
    p0 = [1., 1.,1.] # initial guess
    result = scipy.optimize.leastsq(residiuals, p0, args=(Z,X,Y))[0]
    normal = result[0:3]
    nn = np.linalg.norm(normal)
    normal = normal / nn
    return normal
'''
def Plane_model_segmentation():

    ###
    cloud = pcl.PointCloud()

    points = np.zeros((15, 3), dtype=np.float32)
    RAND_MAX = 1024.0
    for i in range(0, 15):
        points[i][0] = 1024 * random.random() / (RAND_MAX + 1.0)                    
        points[i][1] = 1024 * random.random() / (RAND_MAX + 1.0)
        points[i][2] = 1.0

    points[0][2] = 2.0
    points[3][2] = -2.0
    points[6][2] = 4.0

    cloud.from_array(points)

    #   std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
    #   for (size_t i = 0; i < cloud->points.size (); ++i)
    #     std::cerr << "    " << cloud->points[i].x << " "
    #                         << cloud->points[i].y << " "
    #                         << cloud->points[i].z << std::endl;
    #
    print('Point cloud data: ' + str(cloud.size) + ' points')
    for i in range(0, cloud.size):
        print('x: ' + str(cloud[i][0]) + ', y : ' +
              str(cloud[i][1]) + ', z : ' + str(cloud[i][2]))

    seg = cloud.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.01)
    seg.set_normal_distance_weight(0.01)
    seg.set_max_iterations(100)
    indices, coefficients = seg.segment()

    #   if (inliers->indices.size () == 0)
    #   {
    #     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    #     return (-1);
    #   }
    #   std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    #                                       << coefficients->values[1] << " "
    #                                       << coefficients->values[2] << " "
    #                                       << coefficients->values[3] << std::endl;
    ###
    if len(indices) == 0:
        print('Could not estimate a planar model for the given dataset.')
        exit(0)

    print('Model coefficients: ' + str(coefficients[0]) + ' ' + str(
        coefficients[1]) + ' ' + str(coefficients[2]) + ' ' + str(coefficients[3]))

    #   std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    #   for (size_t i = 0; i < inliers->indices.size (); ++i)
    #     std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
    #                                                << cloud->points[inliers->indices[i]].y << " "
    #                                                << cloud->points[inliers->indices[i]].z << std::endl;
    ###
    print('Model inliers: ' + str(len(indices)))
    for i in range(0, len(indices)):
        print(str(indices[i]) + ', x: ' + str(cloud[indices[i]][0]) + ', y : ' +
              str(cloud[indices[i]][1]) + ', z : ' + str(cloud[indices[i]][2]))

'''
def find_plane(pcd):
    #xyz = np.asarray(pcd.points)
    xyz = np.asarray(pcd)

    XY = xyz[:, :2]
    
    Z = xyz[:, 2]
    ransac = linear_model.RANSACRegressor(residual_threshold=0.0001)

    ransac.fit(XY, Z)
    a, b = ransac.estimator_.coef_  # 係数
    d = ransac.estimator_.intercept_  # 切片

    return a, b, d  # Z = aX + bY + d


def angle_rotate(a, b, d):
    x = np.arange(30)
    y = np.arange(30)
    X, Y = np.meshgrid(x, y)
    Z = a * X + b * Y + d
    rad = math.atan2(Y[1][0] - Y[0][0], (Z[1][0] - Z[0][0]))
    return rad - math.pi


def show_graph(X, Y, Z):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot_surface(X, Y, Z)

    plt.show()