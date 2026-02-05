from networkx import Graph, shortest_path
import numpy as np
from robot import Simple_Manipulator as Robot
import typing
import networkx as nx
from scipy.spatial import KDTree
from networkx.algorithms.shortest_paths.weighted import dijkstra_path


def M3(robot: Robot, samples: np.array, G: Graph, q_start: np.array, q_goal: np.array) -> typing.Tuple[np.array, bool]:
    """ Find a path from q_start to q_goal using the PRM roadmap

    Parameters
    ----------
    robot : Robot
        our robot object
    samples : np.array
        num_samples x 4 numpy array of nodes/vertices in the roadmap
    G : Graph
        An undirected NetworkX graph object with the number of nodes equal to num_samples, 
        and weighted edges indicating collision free connections in the robot's configuration space
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


    # # #student code start here
    # # raise NotImplementedError

    # Add start and goal configurations as special nodes in the graph
    G.add_node('start', config=q_start)
    G.add_node('goal', config=q_goal)

    # Use KDTree for efficient nearest neighbor search among the samples
    tree = KDTree(samples)

    # Function to connect a special node (start/goal) to its nearest neighbors in the graph
    def connect_node(node_key: str, num_connections: int = 5):
        distances, indices = tree.query(G.nodes[node_key]['config'], k=num_connections)
        for i, index in enumerate(indices):
            if robot.check_edge(G.nodes[node_key]['config'], samples[index]):
                G.add_edge(node_key, index, weight=distances[i])

    # Connect start and goal to the graph
    connect_node('start')
    connect_node('goal')

    # Attempt to find a path from 'start' to 'goal' in the graph
    try:
        # Use Dijkstra's algorithm to find the shortest path
        path_indices = dijkstra_path(G, 'start', 'goal', weight='weight')
        # Construct the path from the indices, converting special nodes back to their configurations
        path = np.array([G.nodes[idx]['config'] if idx in G.nodes else idx for idx in path_indices])
        path_found = True
    except nx.NetworkXNoPath:
        path = np.array([])
        path_found = False

    return path, path_found