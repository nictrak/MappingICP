from icp_python_robotics import icp_matching


def icp_animated(previous_points, current_points):
    rotation_matrix, transformation_matrix = icp_matching(previous_points, current_points)
    print("R:", rotation_matrix)
    print("T:", transformation_matrix)
    return rotation_matrix, transformation_matrix



