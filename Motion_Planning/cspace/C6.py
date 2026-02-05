import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
from q2poly import q2poly
import shapely
from shapely.geometry import Polygon as Poly_shapely
from shapely import MultiPoint
import typing

from C1 import C1_func




def C6_func(robot: typing.Dict[str, typing.List[float]], q_path: typing.List[np.array], obstacles: typing.List[Polygon]) -> int:
    """Calculate the number of collisions that occur along the path.
    
    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters.
    q_path : typing.List[np.array]
       A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using actual angle values.
    obstacles : typing.List[Polygon]
        A list of polygons representing the obstacles.

    Returns
    -------
    int
        The number of collisions that occur along the path.
    """

    ### Insert your code below: ###
    num_collisions = 0

    for i in range(len(q_path) - 1):
        # Get the configurations
        q_start = q_path[i]
        q_next = q_path[i + 1]
        
        # Get shapes for start and end configurations
        link1_start, link2_start, _, _ = q2poly(robot, q_start)
        link1_end, link2_end, _, _ = q2poly(robot, q_next)
        
        # Combine the start and end shapes to get the swept volume
        swept_volume1 = MultiPoint(list(Poly_shapely(link1_start).exterior.coords) + list(Poly_shapely(link1_end).exterior.coords)).convex_hull
        swept_volume2 = MultiPoint(list(Poly_shapely(link2_start).exterior.coords) + list(Poly_shapely(link2_end).exterior.coords)).convex_hull
        
        # Check for collisions with obstacles
        for obstacle in obstacles:
            # Convert the obstacle to a Shapely polygon
            obstacle_poly = Poly_shapely(obstacle)
            # Check for collisions with the swept volumes of the robot's links
            if swept_volume1.intersects(obstacle_poly) or swept_volume2.intersects(obstacle_poly):
                # Increment the number of collisions
                num_collisions += 1
                break  # Stop checking other obstacles if a collision is found
                
    return num_collisions