from typing import Tuple
import numpy as np

global num_iter

num_iter = 4000

def q3(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    '''ß
    Localize a cylinder in the point cloud. Given a point cloud as
    input, this function should locate the position, orientation,
    and radius of the cylinder

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting 100 points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting cylinder center
    axis : np.ndarray
        array of shape (3,) pointing along cylinder axis
    radius : float
        scalar radius of cylinder
    '''


    # ### Enter code below
    # center = np.array([1,0.5,0.1])
    # axis = np.array([0,0,1])
    # radius = 0.05
    # return center, axis, radius

    # Initialize variables  
    best_inliers = []
    best_center = np.zeros(3)
    best_axis = np.zeros(3)
    best_radius = 0

    # Define the number of points
    num_points = P.shape[0]

    for _ in range(num_iter):
        # Randomly select two points and their normals
        index_pair = np.random.choice(range(num_points), 2, replace=False)
        normal1, normal2 = N[index_pair[0]], N[index_pair[1]]

        # Normalize the normals
        normal1 /= np.linalg.norm(normal1)
        normal2 /= np.linalg.norm(normal2)

        # Calculate the axis as the cross product of the two normals
        axis = np.cross(normal1, normal2)
        axis /= np.linalg.norm(axis)  # Normalize the axis

        # Choose a random radius within the specified range
        radius = np.random.uniform(0.05, 0.1)

        # # Calculate a point on the axis (midpoint of the line connecting the two points)
        initial_center = P[index_pair[0]] + radius * normal1

        # # Adjust center to sink into the plane 
        # center = initial_center - np.dot(initial_center - P[index_pair[1]], axis) * axis
        center = initial_center - (np.dot(initial_center - P[index_pair[1]], axis) * axis * 0.5)

        # sink the cylinder more down in x axis
        center[2] = center[2] - 2 * radius


        # Computing the projection of the P onto the plane perpendicular to the axis
        n = np.eye(3) - np.outer(axis, axis)
        projected_points = np.matmul(n, P.T).T
        projected_center = np.matmul(n, center.T).T
        d = np.linalg.norm(projected_points - projected_center, axis=1)

        # Finding the inliers
        inliers = np.where(np.logical_and(np.abs(d) < radius + 0.00005,
                                            np.abs(d) > radius - 0.00005))[0]

        # Update the best model if more inliers are found
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_center = center
            best_axis = axis
            best_radius = radius

    # Return the best model
    return best_center, best_axis, best_radius

