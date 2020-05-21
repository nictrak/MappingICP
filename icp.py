from icp_python_robotics import icp_matching
import math


def icp(previous_points, current_points):
    rotation_matrix, transformation_matrix = icp_matching(previous_points, current_points)
    return rotation_matrix, transformation_matrix


def rotation_to_degrees(rotation):
    return math.fabs(math.asin(rotation[0][1]) * 180 / math.pi)





