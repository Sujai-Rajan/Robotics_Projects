import typing
import numpy as np
from networkx import Graph
from robot import Simple_Manipulator as Robot
import networkx as nx
from scipy.spatial import KDTree

from M1 import M1

def M2(robot: Robot, num_samples: int, num_neighbors: int) -> typing.Tuple[np.array, Graph]:
    """ Implement the PRM algorithm

    Parameters
    ----------
    robot : Robot
        our pybullet robot class
    num_samples : int
        number of samples in PRM
    num_neighbors : int
        number of closest neighbors to consider in PRM

    Returns
    -------
    typing.Tuple[np.array, Graph]
        np.array: 
            num_samples x 4 numpy array, sampled configurations in the roadmap (vertices)
        G: 
            a NetworkX graph object with weighted edges indicating the distance between connected nodes in the joint configuration space.
            This should be impelemented as an undirected graph.
    """

    # HINTS
    # useful functions and parameters
    # robot.lower_lims, robot.upper_lims -> Joint Limits
    # robot.check_edge() -> check the linear path between 2 joint configurations for collisions
    
    ### student code start here
    # raise NotImplementedError

      
# Initialize the PRM graph
    G = nx.Graph()
    # start_config = robot.start # Start configuration
    # goal_config = robot.goal # Goal configuration
    start_sample = np.array([0., 0., 0., 0.])
    goal_sample = np.array([0, -np.pi, 0, -np.pi])

    # Sample configurations, including start and goal, ensuring they are collision-free
    # samples = [start_config, goal_config]
    samples = [start_sample, goal_sample]
    while len(samples) < num_samples :  # Subtract 2 for start and goal
        sample = M1(robot.lower_lims, robot.upper_lims, 1)[0]
        if not robot.is_in_collision(sample):
            samples.append(sample)
    samples = np.array(samples)

    # Use KDTree for efficient nearest neighbor search
    tree = KDTree(samples)

    # Add samples as nodes and connect nearest neighbors with collision-free edges
    for i, sample in enumerate(samples):
        G.add_node(i, config=sample)
        distances, indices = tree.query(sample, k=num_neighbors + 1)

        for j, index in enumerate(indices):
            if i == index:  # Skip the self-reference
                continue
            neighbor = samples[index]
            
            # Check for collision-free path with specified resolution
            if robot.check_edge(sample, neighbor, resolution=10):  # Adjust resolution as needed
                distance = distances[j]
                G.add_edge(i, index, weight=distance)

    return samples, G


