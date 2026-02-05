import numpy as np
from shapely.geometry import Polygon as Polygon_shapely
import typing

def q2poly(robot: typing.Dict[str, typing.List[float]], robot_configuration: typing.List[float]) -> typing.Tuple[np.array, np.array, np.array, np.array]:
    """ A function that takes in the robot's parameters and a configuration and 
    returns the vertices of the robot's links after transformation and the pivot points of the links after transformation

    Parameters
    ----------
    robot : typing.dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    q : typing.List[float]
        A 2-element list representing the configuration of the robot

    Returns
    -------
    typing.Tuple[np.array, np.array, np.array, np.array]
        np.array: 
            a numpy array representing the vertices of the first link of the robot after transformation
        np.array: 
            a numpy array representing the vertices of the second link of the robot after transformation
        np.array: 
            a numpy array representing the pivot point of the first link of the robot after transformation
        np.array: 
            a numpy array representing the pivot point of the second link of the robot after transformation
    """


    ### Insert your code below: ###
    
    # Initialize the variables
    shape_1 = np.zeros((len(robot["link1"]),2))
    shape_2 = np.zeros((len(robot["link2"]),2))
    pivot_1 = np.zeros((2,))
    pivot_2 = np.zeros((2,))

    # Extract the robot parameters
    link_1 = np.array(robot["link1"])
    link_2 = np.array(robot["link2"])
    pivot_1 = np.array(robot["pivot1"])

    # Calculate the robot configuration for the second link
    robot_configuration = np.array([-robot_configuration[0], -robot_configuration[1]])
    robot_configuration_link_2 = robot_configuration[0] + robot_configuration[1]

    # Calculate the transformation matrix
    transformation_matrix_1 = np.array([[np.cos(robot_configuration[0]), -np.sin(robot_configuration[0])],[np.sin(robot_configuration[0]), np.cos(robot_configuration[0])]])
    transformation_matrix_2 = np.array([[np.cos(robot_configuration_link_2), -np.sin(robot_configuration_link_2)],[np.sin(robot_configuration_link_2), np.cos(robot_configuration_link_2)]])

    # Calculate the shape of the first link
    shape_1 = link_1 @ transformation_matrix_1 + pivot_1

    # Calculate the pivot point of the second link
    pivot_2 = [2.1 * np.cos(-robot_configuration[0]) + pivot_1[0], 2.1 * np.sin(-robot_configuration[0]) + pivot_1[1]]

    # Calculate the shape of the second link
    shape_2 = link_2 @ transformation_matrix_2 + pivot_2


    return shape_1, shape_2, pivot_1, pivot_2