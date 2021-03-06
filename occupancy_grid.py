import math
import numpy as np
from functools import reduce

IDENTITY = np.array([[1, 0],
                     [0, 1]])


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
    c = origin[1] - slope * origin[0]
    # define line space for x axis
    x_space = (np.linspace(origin[0], point[0], num))[:-1]
    # calculate y space from x space
    y_space = np.array(list(map(lambda a: slope * a + c, x_space)))
    # wrap up result
    data = np.array([x_space, y_space])
    return data


def project_points_onto_origin(points_t, length, origin=np.array([0, 0])):
    """
    :param points: 2D points array
    :param length: length between points in line space
    :param grid_x: the line space for x axis to make histogram
    :param grid_y: the line space for y axis to make histogram
    :param origin: the origin which we want to be projected onto
    :return: 2D points array of lines which are projection of inputs
    """
    # find distances from origins
    distances = map(lambda point: math.sqrt((point[0]-origin[0])**2 + (point[1]-origin[1])**2), points_t)
    # find number of points for line space
    nums = map(lambda dist: int(dist//length), distances)
    # project all
    projections = reduce(lambda array0, array1: np.concatenate((array0, array1), axis=0), map(lambda p, n: _project_point_onto_origin(p, n, origin).T, points_t, nums)).T
    return projections


def cal_positive_grid(points, grid_x, grid_y, origin=np.array([0, 0]), rotation=IDENTITY):
    if len(points[0]) <= 0:
        transform = points
    else:
        transform_t = transform_origin(points, origin=origin, rotation=rotation)
        transform = transform_t.T
    positive_grid, _, _ = np.histogram2d(transform[0], transform[1], bins=[grid_x, grid_y])
    return positive_grid * 0.5


def cal_negative_grid(points, length, grid_x, grid_y, origin=np.array([0, 0]), rotation=IDENTITY):
    transform_t = transform_origin(points, origin=origin, rotation=rotation)
    projections = project_points_onto_origin(transform_t, length, origin)
    raw_grid, _, _ = np.histogram2d(projections[0], projections[1], bins=[grid_x, grid_y])
    negative_grid = raw_grid * -0.5
    return negative_grid


def _cal_prop(value):
    ten_expo = 10 ** value
    return ten_expo / (ten_expo + 1)


def cal_occupancy_grid(grid):
    return np.array(list(map(lambda x: x, map(_cal_prop, grid))))


def transform_raw(raw):
    return np.array(list(map(raw_to_point, raw)))


def update_grid(grid, points_t, length, grid_x, grid_y, origin=np.array([0, 0]), rotation=IDENTITY):
    return grid + cal_positive_grid(points_t, grid_x, grid_y, origin, rotation) + cal_negative_grid(points_t, length, grid_x, grid_y, origin, rotation)


def occupancy_grid_to_points(occupancy_grid, min_value, max_value, length, threshold=0.95):
    # TODO
    pass


def transform_origin(points, origin=np.array([0, 0]), rotation=IDENTITY):
    rotated = np.dot(rotation, points)
    transform_t = np.array(list(map(lambda point: point + origin, rotated.T)))
    return transform_t


