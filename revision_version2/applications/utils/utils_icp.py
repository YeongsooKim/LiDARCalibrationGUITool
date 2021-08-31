import numpy as np
from sklearn.neighbors import NearestNeighbors



def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t


def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    assert src.shape == dst.shape

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()


def icp(A, B, 
        init_pose = None, 
        max_iterations = 20, 
        tolerance = 0.001, 
        rm_outlier_dist = 1e3):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
        converged: Convergence of icp algorithm (mean error within the tolerance)
    '''
    assert A.shape == B.shape
    
    converged = False

    # get number of dimensions
    m = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m+1,A.shape[0]))
    dst = np.ones((m+1,B.shape[0]))
    src[:m,:] = np.copy(A.T)
    dst[:m,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0

    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)

        # outlier remove
        idx_src = np.arange(A.shape[0])
        idx_dst = indices

        cond = distances < rm_outlier_dist
        
#         assert np.sum(cond) > 0
        if np.sum(cond) <= 0:
            converged = False
            break

        idx_src = idx_src[cond]
        idx_dst = indices[cond]       

        # compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[:m,idx_src].T, dst[:m,idx_dst].T)

        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            converged = True
            break
        prev_error = mean_error

    # calculate final transformation
    T,_,_ = best_fit_transform(A, src[:m,:].T)

    return T, distances, i, converged


def icp_NM(A, B, 
        init_pose = None, 
        max_iterations = 20, 
        tolerance = 0.001, 
        rm_outlier_dist = -1):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxd numpy array of source mD points
        B: Mxd numpy array of destination mD point
        init_pose: (d+1)x(d+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''
    A, B = random_upsampling(A, B)
    
    T, distances, iterations, converged = icp(A, B,
                                              init_pose, 
                                              max_iterations, 
                                              tolerance, 
                                              rm_outlier_dist)

    return T, distances, iterations, converged


def random_upsampling(A, B):
  assert A.shape[1] == B.shape[1]
  
  n1 = A.shape[0]
  n2 = B.shape[0]
  
  n = n1>n2 and n1 or n2
    
  if not n1 == n: # A
    sample = np.random.choice(n1, n-n1)
    a = A[sample]    
    AA = np.concatenate([A, a], axis=0)
    
  else:
    AA = A
    
  if not n2 == n: # B
    sample = np.random.choice(n2, n-n2)
    b = B[sample]    
    BB = np.concatenate([B, b], axis=0)
    
  else:
    BB = B
    
  return AA, BB

    
def test_ramdom_upsamling():
  N = 10
  dim = 3
  A = np.random.rand(N, dim)
  B = np.random.rand(int(N * 0.8), dim)

  A, B = random_upsampling(A, B)

if __name__ == '__main__':
  test_ramdom_upsamling()