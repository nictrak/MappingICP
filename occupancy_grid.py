import math
import numpy as np
from functools import reduce


def raw_to_point(raw):
    # define meaning of raw data
    deg = raw[1]
    distance = raw[2]
    # deg to rad
    rad = deg * math.pi / 180
    # calculate point
    x = distance * math.cos(rad)
    y = distance * math.sin(rad)
    # return
    return x, y


def _project_point_onto_origin(point, num, origin=np.array([0, 0])):
    """
    :param point: 2D point
    :param num: number of points in line space
    :param origin: origin point which we want to project to
    :return: array pf point which represent line space
    """
    # calculate slope
    slope = reduce(lambda a, b: b / a, map(lambda a, b: a - b, point, origin))
    # define line space for x axis
    x_space = (np.linspace(0, point[0], num))[:-1]
    # calculate y space from x space
    y_space = np.array(list(map(lambda a: slope * a, x_space)))
    # wrap up result
    data = np.array([x_space.T, y_space.T])
    return data


def project_points_onto_origin(points, length, grid_x, grid_y, origin=np.array([0, 0])):
    """
    :param points: 2D points array
    :param length: length between points in line space
    :param grid_x: the line space for x axis to make histogram
    :param grid_y: the line space for y axis to make histogram
    :param origin: the origin which we want to be projected onto
    :return: 2D points array of lines which are projection of inputs
    """
    points_t = points.T
    # point - origin
    translation_matrix = map(lambda point: point - origin, points_t)
    # find distances from origins
    distances = map(lambda trans: math.sqrt(trans[0]**2 + trans[1]**2), translation_matrix)
    # find number of points for line space
    nums = map(lambda dist: dist/length, distances)
    # project all
    projections = map(lambda p, n: _project_point_onto_origin(p, n, origin), points_t, nums)
    return projections


def cal_positive_grid(points, grid_x, grid_y):
    positive_grid, _, _ = np.histogram2d(points[0], points[1], bins=[grid_x, grid_y])
    return positive_grid


def cal_negative_grid(points, grid_x, grid_y):
    # TODO
    pass


def _cal_prop(value):
    ten_expo = 10 ** value
    return ten_expo / (ten_expo + 1)


def cal_occupancy_grid(grid):
    return np.array(list(map(lambda x: x, map(_cal_prop, grid))))


def update_grid(grid, data, grid_x, grid_y):
    points = np.array(list(map(raw_to_point, data))).T
    return grid + cal_positive_grid(points, grid_x, grid_y)
