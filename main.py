from rplidar import RPLidar
import matplotlib.pyplot as plt
from matplotlib import colors
from occupancy_grid import *
from map_saver import *
from icp import *
import random

PORT = 'COM24'
MIN_RANGE = -6000
MAX_RANGE = 6000
GRID_LENGTH = 10
GRID_POINTS = (MAX_RANGE-MIN_RANGE)//GRID_LENGTH
TRACKER = np.array([1000, 0])

def _do_random_remove(array, length):
    if len(array) <= length:
        return array
    rand = random.randint(0, len(array)-1)
    return _do_random_remove(np.concatenate((array[:rand], array[rand + 1:]), axis=0), length)


def random_filter(previous, current):
    len_pre = len(previous)
    len_cur = len(current)
    diff = len_cur - len_pre
    if diff > 20 or diff < -20:
        return np.array([]), np.array([])
    if diff == 0:
        return previous, current
    if diff > 0:
        return previous, np.array(_do_random_remove(current, len_pre))
    if diff < 0:
        return np.array(_do_random_remove(previous, len_cur)), current
    return np.array([]), np.array([])


def main():
    lidar = RPLidar(PORT)
    # check RPLidar
    info = lidar.get_info()
    print(info)
    health = lidar.get_health()
    print(health)
    # setup value
    points = np.array([[], []])
    previous = np.array([[], []])
    rotate = IDENTITY
    slide = np.array([0, 0])
    grid_x = np.linspace(MIN_RANGE, MAX_RANGE, GRID_POINTS)
    grid_y = np.linspace(MIN_RANGE, MAX_RANGE, GRID_POINTS)
    grid = cal_positive_grid(points, grid_x, grid_y)
    fig = plt.figure()
    # loop while mapping
    for i, scan in enumerate(lidar.iter_scans(min_len=60)):
        raw = filter(lambda element: element[1] >= 270 or element[1] <= 90, scan)
        points = transform_raw(raw)
        pre, cur = random_filter(previous, points)
        if len(pre) > 0 and len(cur) > 0 and i % 5 == 0:
            is_change = False
            rotation, transform = icp(pre.T, cur.T)
            rotation_to_degrees(rotation)
            if rotation_to_degrees(rotation) >= 1:
                rotate = np.dot(rotation, rotate)
                is_change = True
            if math.sqrt(transform[0] ** 2 + transform[1] ** 2) >= 5:
                slide = slide + np.dot(rotate, transform)
                is_change = True
            if is_change:
                previous = points
        grid = update_grid(grid, points.T, GRID_LENGTH, grid_x, grid_y, origin=slide, rotation=rotate)
        # make occupancy grid
        # illustrated
        if i % 10 == 0:
            occupancy_grid = cal_occupancy_grid(grid)
            plt.clf()
            plt.pcolormesh(grid_x, grid_y, occupancy_grid)
            plt.plot(slide[1], slide[0], 'ro')
            tracker = np.dot(rotate, TRACKER)
            plt.plot(slide[1]+tracker[1], slide[0]+tracker[0], 'bo')
            plt.pause(0.01)
        if i == 2:
            previous = points
    # stop RPlidar
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    # file save
    save_override_map(occupancy_grid)


if __name__ == '__main__':
    main()