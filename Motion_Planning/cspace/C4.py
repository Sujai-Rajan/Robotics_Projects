import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
import typing

def C4_func(distances: np.array,q_grid: np.array, q_start: np.array) -> typing.List[np.array]:
    """Using the distance array from C3, find the optimal path from the start configuration to the goal configuration (zero value).

    Parameters
    ----------
    distances : np.array
        A 2D numpy array representing the distance from each cell in the configuration space to the goal configuration.
        This is given by C3 
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.
    q_start : np.array
        A 2 x 1 numpy array representing the start configuration of the robot in the format of [q1, q2].

    Returns
    -------
    typing.List[np.array]
        A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using indices of q_grid.
        Example: [ [q1_0 , q2_0], [q1_1, q2_1], .... ]
    """
    
    ### Insert your code below: ###
    # Find the start index
    start_idx = [np.argmin(np.abs(q_grid - q_start[0])), np.argmin(np.abs(q_grid - q_start[1]))]
    # Create a list to store the path
    current = np.array(start_idx)
    path = [current]
    # Create a list to store the checked cells
    checked = np.zeros(distances.shape)

    # If the goal is not reachable return an empty list
    if not np.argwhere(distances == 2).size: return []

    # While the current cell is not the goal cell
    while not np.array_equal(current, np.argwhere(distances == 2)[0]):
        # Create a list to store the neighbors
        neighbors = []

        # Loop through the 3x3 grid around the current cell
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if dx == 0 and dy == 0: continue
                neighbor = (current[0] + dx, current[1] + dy)
                # If the neighbor is within the bounds of the distance array and it is not an obstacle
                if 0 <= neighbor[0] < distances.shape[0] and 0 <= neighbor[1] < distances.shape[1] and distances[neighbor] != 1 and not checked[neighbor]:
                    # Add the neighbor to the list of neighbors
                    neighbors.append(neighbor)

        # If there are no neighbors, return an empty list
        if not neighbors: return []

        dists = np.inf
        next_idx = None
        # Loop through the neighbors
        for neighbor in neighbors:
            # If the neighbor has not been checked
            if not checked[neighbor]:
                # If the distance to the goal is less than the current minimum distance
                if distances[neighbor] < dists:
                    # Update the minimum distance and the next index
                    dists = distances[neighbor]
                    next_idx = neighbor

        # Update the current cell
        current = next_idx
        # Add the current cell to the path
        path.append(current)
        # Mark the current cell as checked
        checked[current[0], current[1]] = 1

    return path


