import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
import typing


def C3_func(robot: typing.Dict[str, typing.List[float]], cspace: np.array,q_grid: np.array, q_goal: np.array) -> np.array:
    """Create a new 2D array that shows the distance from each point in the configuration space to the goal configuration.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    cspace : np.array
        The configuration space of the robot given by C2. The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.
    q_goal : np.array
        A 2 x 1 numpy array representing the goal configuration of the robot in the format of [q1, q2].

    Returns
    -------
    np.array
       A 2D numpy array representing the distance from each cell in the configuration space to the goal configuration. 
       The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    """

   
    ### Insert your code below: ###

    # Initialize the distance array with same shape as the configuration space
    distances = np.zeros_like(cspace)  

    # Convert the goal configuration to grid indices
    goal_indices = np.array([np.argmin(np.abs(q_grid - q_goal[0])), np.argmin(np.abs(q_grid - q_goal[1]))])

    distances[cspace == 1] = 1
    distances[goal_indices[0], goal_indices[1]] = 2
    # distances[tuple(goal_indices)] = 2

    # Queue to manage cells to visit, starting with the goal
    visit_queue = [(goal_indices[0], goal_indices[1])]

    neighbors = []

    while visit_queue:
        current = visit_queue.pop(0)
        current_x, current_y = current

        # Check for neighbors
        neighbors = [(current_x + dx , current_y + dy ) for dx in range(-1, 2) for dy in range(-1, 2) if dx != 0 or dy != 0]


        # # Check all 4 adjacent cells 
        # for dx in range(-1, 2):
        #     for dy in range(-1, 2):
        #         # check the current cell
        #         if dx != 0 or dy != 0:
        #             neighbor = (current[0] + dx, current[1] + dy)
        #             neighbors.append(neighbor)

        for neighbor in neighbors:
            # Ensure the neighbor is within bounds
            if 0 <= neighbor[0] < cspace.shape[0] and 0 <= neighbor[1] < cspace.shape[1] and distances[neighbor] == 0 and cspace[neighbor] == 0:
                distances[neighbor[0],neighbor[1]] = distances[current_x, current_y] + 1
                visit_queue.append((neighbor[0], neighbor[1]))

    return distances