from rplidar import RPLidar
import matplotlib.pyplot as plt
from matplotlib import colors
from occupancy_grid import *
from map_saver import *

PORT = 'COM24'
MIN_RANGE = -8000
MAX_RANGE = 8000
GRID_LENGTH = 10
GRID_POINTS = (MAX_RANGE-MIN_RANGE)//GRID_LENGTH


def main():
    lidar = RPLidar(PORT)
    # check RPLidar
    info = lidar.get_info()
    print(info)
    health = lidar.get_health()
    print(health)
    # setup value
    points = np.array([[], []])
    grid_x = np.linspace(MIN_RANGE, MAX_RANGE, GRID_POINTS)
    grid_y = np.linspace(MIN_RANGE, MAX_RANGE, GRID_POINTS)
    grid = cal_positive_grid(points, grid_x, grid_y)
    # loop while mapping
    for i, scan in enumerate(lidar.iter_scans()):
        raw = filter(lambda element: element[1] >= 270 or element[1] <= 90, scan)
        points = transform_raw(raw)
        grid = update_grid(grid, points.T, GRID_LENGTH, grid_x, grid_y)
        if i > 50:
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