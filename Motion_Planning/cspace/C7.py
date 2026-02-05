import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
global padding_value
padding_value = 1

def C7_func(cspace: np.array) -> np.array:
    """Pad the configuration space by one grid cell.

    Parameters
    ----------
    cspace : np.array
        The origianl configuration space of the robot.

    Returns
    -------
    np.array
        The padded configuration space of the robot.
    """

    ### Insert your code below: ###
    padded_cspace = np.copy(cspace)
    rows, cols = cspace.shape

    # Loop through each cell in the configuration space
    for i in range(rows):
        for j in range(cols):
            # Check if the current cell is an obstacle
            if cspace[i, j] == padding_value:
                # Pad the adjacent cells, including diagonals
                for di in range(-1, 2):  # -1, 0, 1
                    for dj in range(-1, 2):  # -1, 0, 1
                        # Skip the current cell (no offset)
                        if di == 0 and dj == 0:
                            continue
                        new_i, new_j = i + di, j + dj
                        # Check bounds and update the padded configuration space
                        if 0 <= new_i < rows and 0 <= new_j < cols:
                            padded_cspace[new_i, new_j] = padding_value

    return padded_cspace