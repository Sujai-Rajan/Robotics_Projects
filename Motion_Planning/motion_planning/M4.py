from M1 import M1
import numpy as np
from robot import Simple_Manipulator as Robot
import typing
global goal_sample_rate , step_size , max_iter

step_size=0.2
goal_sample_rate=0.4
max_iter=1000





def M4(robot: Robot, q_start: np.array, q_goal: np.array) -> typing.Tuple[np.array, bool]:
    """Implement RRT algorithm to find a path from q_start to q_goal

    Parameters
    ----------
    robot : Robot
        our robot object
    q_start : np.array
        1x4 numpy array denoting the start configuration
    q_goal : np.array
        1x4 numpy array denoting the goal configuration

    Returns
    -------
    typing.Tuple[np.array, bool]
        np.array:
            Nx4 numpy array containing a collision-free path between
            q_start and q_goal, if a path is found. The first row
            should be q_start, the final row should be q_goal.
        bool:
            Boolean denoting whether a path was found
    """

    # # student work start here
    # raise NotImplementedError


    class Node:
        def __init__(self, point, parent=None):
            self.point = point
            self.parent = parent

    def distance(p1, p2):
        return np.linalg.norm(p1 - p2)

    def steer(from_node, to_point, step_size):
        direction = (to_point - from_node.point) / distance(from_node.point, to_point)
        new_point = from_node.point + step_size * direction
        return Node(new_point, from_node)

    tree = [Node(q_start)]

    for _ in range(max_iter):
        if np.random.rand() < goal_sample_rate:
            q_rand = q_goal
        else:
            q_rand = M1(robot.lower_lims, robot.upper_lims, 1)[0]

        nearest_node = min(tree, key=lambda node: distance(node.point, q_rand))
        new_node = steer(nearest_node, q_rand, step_size)

        if not robot.check_edge(nearest_node.point, new_node.point):
            continue

        tree.append(new_node)

        if distance(new_node.point, q_goal) < step_size:
            if robot.check_edge(new_node.point, q_goal):
                tree.append(Node(q_goal, new_node))

                # Path reconstruction
                path = []
                current_node = tree[-1]
                while current_node.parent is not None:
                    path.append(current_node.point)
                    current_node = current_node.parent
                path.append(q_start)
                return np.array(path[::-1]), True  # Reverse the path

    return np.array([]), False