from rplidar import RPLidar
import matplotlib.pyplot as plt
from matplotlib import colors
from occupancy_grid import *
from map_saver import *
from icp import *
import random

PORT = 'COM24'
MIN_RANGE = -8000
MAX_RANGE = 8000
GRID_LENGTH = 10
GRID_POINTS = (MAX_RANGE-MIN_RANGE)//GRID_LENGTH


def _do_random_remove(array, length):
    if len(array) <= length:
        return array
    rand = random.randint(0, len(array)-1)
    return _do_random_remove(np.concatenate((array[:rand], array[rand + 1:]), axis=0), length)


def random_filter(previous, current):
    len_pre = len(previous)
    len_cur = len(current)
    diff = len_cur - len_pre
    print(len_pre, len_cur, diff)
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
    grid_x = np.linspace(MIN_RANGE, MAX_RANGE, GRID_POINTS)
    grid_y = np.linspace(MIN_RANGE, MAX_RANGE, GRID_POINTS)
    grid = cal_positive_grid(points, grid_x, grid_y)
    # loop while mapping
    for i, scan in enumerate(lidar.iter_scans(min_len=60)):
        raw = filter(lambda element: element[1] >= 270 or element[1] <= 90, scan)
        points = transform_raw(raw)
        grid = update_grid(grid, points.T, GRID_LENGTH, grid_x, grid_y)
        pre, cur = random_filter(previous, points)
        if len(pre) > 0 and len(cur) > 0:
            icp(pre.T, cur.T)
        previous = points
        if i > 10:
            break
    # stop RPlidar
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    # make occupancy grid
    occupancy_grid = cal_occupancy_grid(grid)
    # illustrated
    fig = plt.figure()
    plt.pcolormesh(grid_x, grid_y, occupancy_grid)
    plt.colorbar()
    plt.grid()
    plt.show()
    # file save
    save_override_map(occupancy_grid)


if __name__ == '__main__':
    main()