from typing import Tuple
import numpy as np
import utils
from scipy.spatial import KDTree



global max_iterations , tolerance

max_iterations = 1000
tolerance = 1e-7

def q4_a(M: np.ndarray, D: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Find transformation T such that D = T @ M. This assumes that M and D are
    corresponding (i.e. M[i] and D[i] correspond to same point)

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    use `np.linalg.svd` to perform singular value decomposition
    '''
    
    # ### Enter code below
    # T = np.eye(4)
    # return T


    # Calculate the centroids of M and D
    centroid_M = np.mean(M, axis=0)
    centroid_D = np.mean(D, axis=0)

    # Center M and D by subtracting their respective centroids
    M_centered = M - centroid_M
    D_centered = D - centroid_D

    # Compute the cross-covariance matrix
    H = np.dot(D_centered.T, M_centered)

    # Perform SVD on the cross-covariance matrix
    U, S, Vt = np.linalg.svd(H)

    # Ensure a right-handed coordinate system (prevent reflection)
    R = np.dot(U, Vt)
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = np.dot(U, Vt)

    # Compute the optimal translation vector
    t = centroid_D - np.dot(R, centroid_M)

    # Construct the 4x4 homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T



def q4_c(M: np.ndarray, D: np.ndarray) -> np.ndarray:
    '''
    Solves iterative closest point (ICP) to generate transformation T to best
    align the points clouds: D = T @ M

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    you should make use of the function `q4_a`
    '''

    # ### Enter code below
    # T = np.eye(4)
    # return T

    T = np.eye(4)
    previous_error = np.inf
    error = np.inf
    i=0

    # Construct KD-tree for efficient nearest neighbor search in D
    kd_tree = KDTree(D)

    while i < max_iterations or error > tolerance:
        # Find closest points in D for each point in M using KD-tree
        distances, indices = kd_tree.query(M)
        closest_points_in_D = D[indices]

        # Estimate transformation using closest points
        Temp = q4_a(M, closest_points_in_D)

        # Update M by applying the transformation
        M_homogeneous = np.hstack((M, np.ones((M.shape[0], 1))))
        M = np.dot(Temp, M_homogeneous.T).T[:, :3]

        # Update the final transformation matrix
        T = np.dot(Temp, T)

        # Calculate the mean squared error (MSE) between M and its closest points in D
        curr_error = np.mean(distances ** 2)

        # Check for convergence
        error = np.abs(curr_error - previous_error)
        print(f'Iteration {i}: error = {error:.6f}')

        # Update the error and iteration count
        previous_error = curr_error
        i += 1

        
    # return the final transformation
    return T

    