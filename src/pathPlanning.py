"""
Grid based sweep planner

author: Atsushi Sakai
"""

import math
import os
import sys
from enum import IntEnum

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot

sys.path.append(os.path.join(os.path.dirname(__file__), 'Mapping', 'grid_map_lib')) # os.path.relpath("../../Mapping/grid_map_lib/")
try:
    from grid_map_lib import GridMap
except ImportError:
    raise

do_animation = False


class SweepSearcher:
    class SweepDirection(IntEnum):
        UP = 1
        DOWN = -1

    class MovingDirection(IntEnum):
        RIGHT = 1
        LEFT = -1

    def __init__(self,
                 moving_direction, sweep_direction, x_inds_goal_y, goal_y):
        self.moving_direction = moving_direction
        self.sweep_direction = sweep_direction
        self.turing_window = []
        self.update_turning_window()
        self.x_indexes_goal_y = x_inds_goal_y
        self.goal_y = goal_y

    def move_target_grid(self, c_x_index, c_y_index, grid_map):
        n_x_index = self.moving_direction + c_x_index
        n_y_index = c_y_index

        # found safe grid
        if not grid_map.check_occupied_from_xy_index(n_x_index, n_y_index,
                                                     occupied_val=0.5):
            return n_x_index, n_y_index
        else:  # occupied
            next_c_x_index, next_c_y_index = self.find_safe_turning_grid(
                c_x_index, c_y_index, grid_map)
            if (next_c_x_index is None) and (next_c_y_index is None):
                # moving backward
                next_c_x_index = -self.moving_direction + c_x_index
                next_c_y_index = c_y_index
                if grid_map.check_occupied_from_xy_index(next_c_x_index,
                                                         next_c_y_index):
                    # moved backward, but the grid is occupied by obstacle
                    return None, None
            else:
                # keep moving until end
                while not grid_map.check_occupied_from_xy_index(
                        next_c_x_index + self.moving_direction,
                        next_c_y_index, occupied_val=0.5):
                    next_c_x_index += self.moving_direction
                self.swap_moving_direction()
            return next_c_x_index, next_c_y_index

    def find_safe_turning_grid(self, c_x_index, c_y_index, grid_map):

        for (d_x_ind, d_y_ind) in self.turing_window:

            next_x_ind = d_x_ind + c_x_index
            next_y_ind = d_y_ind + c_y_index

            # found safe grid
            if not grid_map.check_occupied_from_xy_index(next_x_ind,
                                                         next_y_ind,
                                                         occupied_val=0.5):
                return next_x_ind, next_y_ind

        return None, None

    def is_search_done(self, grid_map):
        for ix in self.x_indexes_goal_y:
            if not grid_map.check_occupied_from_xy_index(ix, self.goal_y,
                                                         occupied_val=0.5):
                return False

        # all lower grid is occupied
        return True

    def update_turning_window(self):
        # turning window definition
        # robot can move grid based on it.
        self.turing_window = [
            (self.moving_direction, 0.0),
            (self.moving_direction, self.sweep_direction),
            (0, self.sweep_direction),
            (-self.moving_direction, self.sweep_direction),
        ]

    def swap_moving_direction(self):
        self.moving_direction *= -1
        self.update_turning_window()

    def search_start_grid(self, grid_map):
        x_inds = []
        y_ind = 0
        if self.sweep_direction == self.SweepDirection.DOWN:
            x_inds, y_ind = search_free_grid_index_at_edge_y(
                grid_map, from_upper=True)
        elif self.sweep_direction == self.SweepDirection.UP:
            x_inds, y_ind = search_free_grid_index_at_edge_y(
                grid_map, from_upper=False)

        if self.moving_direction == self.MovingDirection.RIGHT:
            return min(x_inds), y_ind
        elif self.moving_direction == self.MovingDirection.LEFT:
            return max(x_inds), y_ind

        raise ValueError("self.moving direction is invalid ")


def find_sweep_direction_and_start_position(ox, oy):
    # find sweep_direction
    max_dist = 0.0
    vec = [0.0, 0.0]
    sweep_start_pos = [0.0, 0.0]
    for i in range(len(ox) - 1):
        dx = ox[i + 1] - ox[i]
        dy = oy[i + 1] - oy[i]
        d = np.hypot(dx, dy)

        if d > max_dist:
            max_dist = d
            vec = [dx, dy]
            sweep_start_pos = [ox[i], oy[i]]

    return vec, sweep_start_pos


def convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_position):
    tx = [ix - sweep_start_position[0] for ix in ox]
    ty = [iy - sweep_start_position[1] for iy in oy]
    th = math.atan2(sweep_vec[1], sweep_vec[0])
    rot = Rot.from_euler('z', th).as_matrix()[0:2, 0:2]
    converted_xy = np.stack([tx, ty]).T @ rot

    return converted_xy[:, 0], converted_xy[:, 1]


def convert_global_coordinate(x, y, sweep_vec, sweep_start_position):
    th = math.atan2(sweep_vec[1], sweep_vec[0])
    rot = Rot.from_euler('z', -th).as_matrix()[0:2, 0:2]
    converted_xy = np.stack([x, y]).T @ rot
    rx = [ix + sweep_start_position[0] for ix in converted_xy[:, 0]]
    ry = [iy + sweep_start_position[1] for iy in converted_xy[:, 1]]
    return rx, ry


def search_free_grid_index_at_edge_y(grid_map, from_upper=False):
    y_index = None
    x_indexes = []

    if from_upper:
        x_range = range(grid_map.height)[::-1]
        y_range = range(grid_map.width)[::-1]
    else:
        x_range = range(grid_map.height)
        y_range = range(grid_map.width)

    for iy in x_range:
        for ix in y_range:
            if not grid_map.check_occupied_from_xy_index(ix, iy):
                y_index = iy
                x_indexes.append(ix)
        if y_index:
            break

    return x_indexes, y_index


def setup_grid_map(ox, oy, resolution, sweep_direction, offset_grid=10):
    width = math.ceil((max(ox) - min(ox)) / resolution) + offset_grid
    height = math.ceil((max(oy) - min(oy)) / resolution) + offset_grid
    center_x = (np.max(ox) + np.min(ox)) / 2.0
    center_y = (np.max(oy) + np.min(oy)) / 2.0

    grid_map = GridMap(width, height, resolution, center_x, center_y)
    # grid_map.print_grid_map_info()
    grid_map.set_value_from_polygon(ox, oy, 1.0, inside=False)
    grid_map.expand_grid()

    x_inds_goal_y = []
    goal_y = 0
    if sweep_direction == SweepSearcher.SweepDirection.UP:
        x_inds_goal_y, goal_y = search_free_grid_index_at_edge_y(
            grid_map, from_upper=True)
    elif sweep_direction == SweepSearcher.SweepDirection.DOWN:
        x_inds_goal_y, goal_y = search_free_grid_index_at_edge_y(
            grid_map, from_upper=False)

    return grid_map, x_inds_goal_y, goal_y


def sweep_path_search(sweep_searcher, grid_map, grid_search_animation=False):
    # search start grid
    c_x_index, c_y_index = sweep_searcher.search_start_grid(grid_map)
    if not grid_map.set_value_from_xy_index(c_x_index, c_y_index, 0.5):
        print("Cannot find start grid")
        return [], []

    x, y = grid_map.calc_grid_central_xy_position_from_xy_index(c_x_index,
                                                                c_y_index)
    px, py = [x], [y]

    fig, ax = None, None
    if grid_search_animation:
        fig, ax = plt.subplots()
        # for stopping simulation with the esc key.
        fig.canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

    while True:
        c_x_index, c_y_index = sweep_searcher.move_target_grid(c_x_index,
                                                               c_y_index,
                                                               grid_map)

        if sweep_searcher.is_search_done(grid_map) or (
                c_x_index is None or c_y_index is None):
            # print("Done")
            break

        x, y = grid_map.calc_grid_central_xy_position_from_xy_index(
            c_x_index, c_y_index)

        px.append(x)
        py.append(y)

        grid_map.set_value_from_xy_index(c_x_index, c_y_index, 0.5)

        if grid_search_animation:
            grid_map.plot_grid_map(ax=ax)
            plt.pause(1.0)

    grid_map.plot_grid_map()

    return px, py


def planning(ox, oy, resolution,
             moving_direction=SweepSearcher.MovingDirection.LEFT,
             sweeping_direction=SweepSearcher.SweepDirection.DOWN,
             ):
    sweep_vec, sweep_start_position = find_sweep_direction_and_start_position(
        ox, oy)

    rox, roy = convert_grid_coordinate(ox, oy, sweep_vec,
                                       sweep_start_position)

    grid_map, x_inds_goal_y, goal_y = setup_grid_map(rox, roy, resolution,
                                                     sweeping_direction)

    sweep_searcher = SweepSearcher(moving_direction, sweeping_direction,
                                   x_inds_goal_y, goal_y)

    px, py = sweep_path_search(sweep_searcher, grid_map)

    rx, ry = convert_global_coordinate(px, py, sweep_vec,
                                       sweep_start_position)

    # print("Path length:", len(rx))
    # combined = np.column_stack((np.array(rx), np.array(ry)))
    # outF = open("myOutFile.txt", "w")
    # outF.write("type	time	latitude	longitude	name	desc\r")

    # for coords in combined:
    #     outF.write("T\t2021-01-05 03:10:31\t")
    #     outF.write(str(coords[0]))
    #     outF.write("\t")
    #     outF.write(str(coords[1]))
    #     outF.write("\r")

    # outF.close()

    return rx, ry


def main():  # pragma: no cover
    print("start!!")

    # reversed # ox = [-28.1570165,-28.1570618,-28.1569979,-28.1568927,-28.1567361,-28.1564991,-28.1563395,-28.1561964,-28.1560049,-28.1558334,-28.1558275,-28.1558417,-28.1558642,-28.1558642,-28.1558677,-28.1558847,-28.1558918,-28.1558954,-28.1558930,-28.1558942,-28.1559001,-28.1559616,-28.1560172,-28.1561260,-28.1562371,-28.1563638,-28.1564974,-28.1566369,-28.1567835,-28.1568249,-28.1569183,-28.1569514,-28.1569869,-28.1570165]
    # oy = [153.3286216,153.3282085,153.3278491,153.3275769,153.3272148,153.3271008,153.3272497,153.3273717,153.3275085,153.3276266,153.3277258,153.3278599,153.3279780,153.3281608,153.3282748,153.3284129,153.3285806,153.3286999,153.3288408,153.3290379,153.3291948,153.3293584,153.3295019,153.3296588,153.3297728,153.3297138,153.3296763,153.3296494,153.3296401,153.3296285,153.3295923,153.3295064,153.3292208, 153.3286216]
    # reversed # oy = [153.3286216,153.3292208,153.3295064,153.3295923,153.3296285,153.3296401,153.3296494,153.3296763,153.3297138,153.3297728,153.3296588,153.3295019,153.3293584,153.3291948,153.3290379,153.3288408,153.3286999,153.3285806,153.3284129,153.3282748,153.3281608,153.3279780,153.3278599,153.3277258,153.3276266,153.3275085,153.3273717,153.3272497,153.3271008,153.3272148,153.3275769,153.3278491,153.3282085,153.3286216]
    # ox = [0,0,10,10,0]
    # oy = [0,10,10,0,0]

    # ox = [-28.1568582,-28.1568038,-28.1568216,-28.1567388,-28.1566537,-28.1565591,-28.1564881,-28.1564515,-28.1564243,-28.1563770,-28.1563060,-28.1562540,-28.1561937,-28.1561878,-28.1561831,-28.1561878,-28.1561961,-28.1561984,-28.1562114,-28.1562398,-28.1562930,-28.1563486,-28.1563971,-28.1564456,-28.1564574,-28.1564668,-28.1564692,-28.1564597,-28.1564420,-28.1563923,-28.1563663,-28.1563628,-28.1563273,-28.1563060,-28.1562587,-28.1561949,-28.1561369,-28.1561062,-28.1560802,-28.1559939,-28.1559679,-28.1559273,-28.1559899,-28.1561212,-28.1561129,-28.1561034,-28.1560443,-28.1560860,-28.1561413,-28.1561702,-28.1561787,-28.1560974,-28.1560380,-28.1560142,-28.1559320,-28.1558882,-28.1558764,-28.1558811,-28.1558740,-28.1558930,-28.1559048,-28.1558634,-28.1559698,-28.1560526,-28.1561354,-28.1565019,-28.1565817,-28.1566473,-28.1567916,-28.1568849,-28.1569133,-28.1568896,-28.1567750,-28.1566768,-28.1566366,-28.1566177,-28.1566000,-28.1566319,-28.1566591,-28.1567146,-28.1566945,-28.1566579,-28.1566153,-28.1565787,-28.1566082,-28.1566579,-28.1567442,-28.1568530,-28.1567787,-28.1568582]
    # oy = [153.3281786,153.3280364,153.3279103,153.3278849,153.3279841,153.3280552,153.3280994,153.3281182,153.3281893,153.3282523,153.3282630,153.3282295,153.3282604,153.3283221,153.3283717,153.3284374,153.3284816,153.3285259,153.3285889,153.3286225,153.3286493,153.3286064,153.3285527,153.3285728,153.3286064,153.3286386,153.3286815,153.3287311,153.3287660,153.3287660,153.3287941,153.3288437,153.3288706,153.3289282,153.3290167,153.3290972,153.3291763,153.3292568,153.3293024,153.3293198,153.3292340,153.3288005,153.3287241,153.3286275,153.3285591,153.3283365,153.3281246,153.3280076,153.3279529,153.3279081,153.3278515,153.3278351,153.3278387,153.3278960,153.3281152,153.3283258,153.3285196,153.3287053,153.3288113,153.3289118,153.3292096,153.3295542,153.3297165,153.3296984,153.3296723,153.3296360,153.3296253,153.3296226,153.3295945,153.3295636,153.3295261,153.3294993,153.3295381,153.3295247,153.3294952,153.3294376,153.3293651,153.3293048,153.3293383,153.3293504,153.3292793,153.3291385,153.3291372,153.3290983,153.3290379,153.3288984,153.3287751,153.3287616,153.3284714,153.3281786]

    ox = [-28.1567392,-28.1568149,-28.1567688,-28.1567392,-28.1567357,-28.1567365,-28.1567424,-28.1566336,-28.1565603,-28.1565130,-28.1565106,-28.1565036,-28.1564882,-28.1565213,-28.1565650,-28.1566253,-28.1567392]
    oy = [153.3280381,153.3281870,153.3282420,153.3282997,153.3283386,153.3284806,153.3285516,153.3285744,153.3285852,153.3285074,153.3284323,153.3283491,153.3282392,153.3281895,153.3281185,153.3280809,153.3280381]
    resolution = 0.00001
    px, py = planning(ox, oy, resolution)

    # for ipx, ipy in zip(px, py):
    #     print(ipx, ipy)

if __name__ == '__main__':
    main()
