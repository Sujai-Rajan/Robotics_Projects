import numpy as np
from robot import Simple_Manipulator as Robot

def M5(robot: Robot, path: np.array) -> np.array:
    """Smooth the given path

    Parameters
    ----------
    robot : Robot
        our robot object
    path : np.array
        Nx4 numpy array containing a collision-free path between q_start and q_goal

    Returns
    -------
    np.array
        Nx4 numpy array containing a smoothed version of the
        input path, where some unnecessary intermediate
        waypoints may have been removed
    """

    #student work start here
    # raise NotImplementedError

    
    if len(path) <= 2:  # If the path is too short, no smoothing is necessary
        return path

    # Initialize the smoothed path with the first waypoint
    smoothed_path = [path[0]]

    i = 0  # Start index
    while i < len(path) - 1:
        # Look ahead for a direct connection from the current waypoint to further waypoints
        for j in range(len(path) - 1, i, -1):
            # Check for a direct, collision-free path between the current waypoint and the lookahead waypoint
            if robot.check_edge(smoothed_path[-1], path[j]):
                # If a direct connection is possible, add the lookahead waypoint to the smoothed path
                smoothed_path.append(path[j])
                i = j  # Update the current index to the lookahead index
                break
        else:
            # If no direct connection is found, proceed to the next consecutive waypoint
            i += 1

    # Ensure the last waypoint (goal) is included in the smoothed path
    if smoothed_path[-1].tolist() != path[-1].tolist():
        smoothed_path.append(path[-1])

    return np.array(smoothed_path)
