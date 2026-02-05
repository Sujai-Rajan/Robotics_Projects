import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from shapely.geometry import Polygon as Poly
from helper_functions import *
from q2poly import q2poly
import typing





def C2_func(robot: typing.Dict[str, typing.List[float]], cspace: np.array, obstacles: typing.List[Poly], q_grid: np.array) -> np.array:
    """Create the configuration space for the robot with the given obstacles in the given empty cspace array.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    cspace : np.array
        An empty 2D numpy array
    obstacles : typing.List[Polygon_shapely]
        A list of Shapely polygons representing the obstacles
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.

    Returns
    -------
    np.array
        A 2D numpy array representing the updated configuration space.
    """
 
    ## Insert your code below: ###

    # poly1 = Polygon_shapely(shape1_tuple)
    # obs1 = Polygon_shapely(obstable_tuple)

    # Resolution of the configuration space
    cspace_resolution = cspace.shape[0]

    # Inital parameters are set
    grid_size = (cspace_resolution , cspace_resolution)
    # grid_size = (100, 100)

    cspace = np.zeros((grid_size))
    # print(grid_size)
    
    # Define the range of angles for both joints (assuming 0 to 2pi for each joint)
    angles = np.linspace(0, 2 * np.pi, num=grid_size[0])#, endpoint=False)
  
    # Transform the obstacles into polygons
    obstacles = [Poly(obstacle) for obstacle in obstacles]

    # For loop to iterate through the grid
    for i in range(grid_size[0]):
        for j in range(grid_size[1]):

            # Convert the grid indices to joint angles
            q = [angles[i], angles[j]]

            # Transform the robot's links into current configuration
            link1, link2, _, _ = q2poly(robot, q)

            # Create the polygons for the robot's links in the current configuration
            poly1 = Poly(link1)
            poly2 = Poly(link2)

            # Check collision for each link with each obstacle
            collision_found = any(poly1.intersects(obstacle) or poly2.intersects(obstacle) for obstacle in obstacles)

            # Update the configuration space
            cspace[i,j] = collision_found

    return cspace