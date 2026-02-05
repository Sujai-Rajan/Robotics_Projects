from typing import Tuple
import numpy as np

import utils

global num_iterations
num_iterations = 1500



def q1_a(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a least squares plane by taking the Eigen values and vectors
    of the sample covariance matrix

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''

    ### Enter code below

    # To see initial implementation, uncomment the following lines
    # center = np.array([0,1,0])
    # normal = np.array([0,0,1])


    # Center the points around the mean
    center = np.mean(P, axis=0)

    # Find the covariance
    covariance = np.cov(P.T)

    # Apply SVD
    U, S, Vt = np.linalg.svd(covariance)

    # The normal to the plane is the last column of V
    normal = Vt[2]
        
    return normal, center



def q1_c(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a plane using RANSAC

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''
    
    ### Enter code below
    # center = np.array([0,1,0])
    # normal = np.array([0,0,1])
    # return normal, center


    # Initialize variables
    best_inliers = []
    best_normal = None
    best_center = None

    for _ in range(num_iterations):
        # Select 3 random points
        random_indices = np.random.choice(P.shape[0], 3, replace=False)
        sample_points = P[random_indices]

        # Compute the centroid
        centroid = np.mean(sample_points, axis=0)

        # Center the points to the centroid
        centered_points = sample_points - centroid

        # Compute SVD
        U, S, Vt = np.linalg.svd(centered_points)

        # The normal vector is the last column of V
        normal = Vt[-1]

        # Calculate distances of all points to the plane
        distances = np.abs((P - centroid).dot(normal))

        # Determine inliers as points with distance below a threshold
        inliers = np.where(distances < 0.01)[0]

        # Update the best model if the current one has more inliers
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_normal = normal
            best_center = centroid

    return best_normal, best_center

    