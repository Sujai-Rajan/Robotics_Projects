from typing import Tuple
import numpy as np

global num_iterations
num_iterations = 1500

def q2(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, float]:
    '''
    Localize a sphere in the point cloud. Given a point cloud as
    input, this function should locate the position and radius
    of a sphere

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting sphere center
    radius : float
        scalar radius of sphere
    '''

    # ### Enter code below
    # center = np.array([1,0.5,0.1])
    # radius = 0.05
    # return center, radius

    
    num_points = P.shape[0]
    best_inliers = []
    best_center = np.zeros((3, 1))
    best_radius = 0

    for _ in range(num_iterations):
        # Pick a point randomly
        random_idx = np.random.choice(num_points)
        random_point = P[random_idx]
        random_normal = N[random_idx]

        # Ensure the normal is a unit vector
        random_normal /= np.linalg.norm(random_normal)

        # Sample a radius between 5cm and 11cm
        curr_radius = np.random.uniform(0.05, 0.11)

        # Compute the center of the sphere
        curr_center = random_point + curr_radius * random_normal

        # Calculate the distances from the center to all points
        distances = np.linalg.norm(P - curr_center, axis=1)

        # Find inliers based on distance to the sphere's surface
        inlier_threshold = 0.002  # Adjust as necessary
        inliers = np.where(np.abs(distances - curr_radius) < inlier_threshold)[0]

        # Update the best model if more inliers are found
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_center = curr_center
            best_radius = curr_radius

    return best_center.reshape((3,)), best_radius