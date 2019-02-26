from Utils.utils import *
from time import time

"""This module defines the simulated Robot class."""

class Robot:
    """
    This class is the simulation robot.
    """
    def __init__(self, exploration_status, facing, discovered_map, real_map):
        """Initialize the robot."""
        self.exploration_status = exploration_status
        self.center = START
        self.facing = facing
        self.discovered_map = discovered_map
        self.real_map = real_map
        self._sensors = [
            {"mount_loc": NWS, "facing": WEST, "range": 2, "blind_spot": 0},
            {"mount_loc": WS, "facing": WEST, "range": 2, "blind_spot": 0},
            {"mount_loc": NWS, "facing": NORTH, "range": 4, "blind_spot": 0},
            {"mount_loc": NS, "facing": NORTH, "range": 2, "blind_spot": 0},
            {"mount_loc": NES, "facing": NORTH, "range": 2, "blind_spot": 0},
            {"mount_loc": NES, "facing": EAST, "range": 4, "blind_spot": 0}
        ]

    def _mark_explored(self, cell_index):
        """
        Discover the cell if it is not already explored.
        Update exploration status as 1
        Set the cell in discovered_map as the one in real_map

        :param cell_index: cell index.
        :return: N/A
        """
        y, x = get_matrix_coords(cell_index)

        # if exploration_status not 0
        if not self.exploration_status[y][x]:
            self.exploration_status[y][x] = 1
            self.discovered_map[y][x] = self.real_map[19 - y][x]
            return get_grid_index(y, x), self.discovered_map[y][x]
        return None, None

    def in_efficiency_limit(self):
        """
        Check if the robot is in a position that would indicate that it had just moved past a 1-width obstacle
        that is against the wall of the maze.

        :return: True if the above condition is fulfilled, false otherwise.
        """
        if (self.center in E_LIMITS[NORTH] and self.facing == EAST) \
                or (self.center in E_LIMITS[EAST] and self.facing == SOUTH) \
                or (self.center in E_LIMITS[SOUTH] and self.facing == WEST) \
                or (self.center in E_LIMITS[WEST] and self.facing == NORTH):
            return True

        return False

    def mark_robot_standing(self):
        """
        Mark the area the robot is standing on as explored

        :return: N/A
        """
        robot_cells = get_robot_cells(self.center)
        updated_cells = {}
        for cell in robot_cells:
            # grid index, 0/1
            updated_cell, value = self._mark_explored(cell)
            if updated_cell is not None:
                updated_cells[updated_cell] = value

        return updated_cells

    def get_completion_percentage(self):
        """
        Calculate how much of the maze the robot has explored in percentage.

        :return: The percentage of the map the robot has explored.
        """
        count = 0
        for row in self.exploration_status:
            for i in row:
                count += i

        return (count / (NUM_COLS * NUM_ROWS)) * 100.0

    def is_complete(self, explore_limit, start_time, time_limit):
        """
        Check if the exploration is complete based on the exploration limit and the time limit.

        :param explore_limit: The percentage of the maze up to which the robot is allowed to explore.
        :param start_time: The start time of the exploration
        :param time_limit: The maximum time that the robot was allowed to explore until.
        :return: True if the exploration should be stopped, false otherwise.
        """
        return self.get_completion_percentage() >= float(explore_limit) \
            or float(time() - start_time >= time_limit)

    def turn_robot(self, direction):
        """
        Turn the robot in a chosen direction.

        :param direction: The direction to turn (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: N/A
        """
        self.facing = (self.facing + direction) % 4
        print('turn_robot facing: {}'.format(self.facing))

    def move_robot(self, direction):
        """
        Move the robot 1 step in a chosen direction. Assume step is not an obstacle.

        :param direction: The direction to move (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: Any cells that the robot has stepped on that it had not yet before.
        """
        self.turn_robot(direction)

        if self.facing == NORTH:
            self.center += ROW_LENGTH
        elif self.facing == EAST:
            self.center += 1
        elif self.facing == SOUTH:
            self.center -= ROW_LENGTH
        elif self.facing == WEST:
            self.center -= 1

        updated_cells = self.mark_robot_standing()

        return updated_cells

    def check_free(self, direction):
        """
        Check if the adjacent cells in the chosen direction have obstacles.

        :param direction: he direction to check (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: true if the robot is able to take one step in that direction, false otherwise
        """

        print('Check Free......')
        print('Direction: {}'.format(MOVEMENTS[direction]))

        true_bearing = (self.facing + direction) % 4
        robot_cells = get_robot_cells(self.center)

        try:
            if true_bearing == NORTH:
                y, x = get_matrix_coords(robot_cells[0])
                y += 1
                print('Cell to check : {}'.format((x, y, x+1, y, x+2, y)))
                if y < 0 or x < 0:
                    raise IndexError
                is_free = not (self.discovered_map[y][x] == 1 or self.discovered_map[y][x + 1] == 1
                            or self.discovered_map[y][x + 2] == 1)
                print('North: ' + str(is_free))
                return is_free
            elif true_bearing == EAST:
                y, x = get_matrix_coords(robot_cells[2])
                x += 1
                print('Cell to check : {}'.format((x, y, x, y-1, x, y-2)))
                if y < 2 or x < 0:
                    raise IndexError
                is_free = not (self.discovered_map[y][x] == 1 or self.discovered_map[y - 1][x] == 1
                            or self.discovered_map[y - 2][x] == 1)
                print('East: ' + str(is_free))
                return is_free
            elif true_bearing == SOUTH:
                y, x = get_matrix_coords(robot_cells[6])
                y -= 1
                print('Cell to check : {}'.format((x, y, x+1, y, x+2, y)))
                if y < 0 or x < 0:
                    raise IndexError
                is_free = not (self.discovered_map[y][x] == 1 or self.discovered_map[y][x + 1] == 1
                            or self.discovered_map[y][x + 2] == 1)
                print('South: ' + str(is_free))
                return is_free
            elif true_bearing == WEST:
                y, x = get_matrix_coords(robot_cells[0])
                x -= 1
                print('Cell to check : {}'.format((x, y, x, y-1, y-2, x)))
                if y < 2 or x < 0:
                    raise IndexError
                is_free = not (self.discovered_map[y][x] == 1 or self.discovered_map[y - 1][x] == 1
                            or self.discovered_map[y - 2][x] == 1)
                print('West: ' + str(is_free))
                return is_free
        except IndexError:
            return False

    def get_sensor_readings(self):
        """
        Get simulated sensor readings by comparing the cells that are to be explored
        by the virtual sensors against the map provided.
        Also mark the cells in sensors' range as explored.

        :return: The updated cell values and indexes.
        """
        updated_cells = {}

        for sensor in self._sensors:
            true_facing = (sensor["facing"] + self.facing) % 4

            if sensor["mount_loc"] != CS:
                offset = self.facing * 2
                true_mounting = (sensor["mount_loc"] + offset) % 8
            else:
                true_mounting = CS

            robot_cells = get_robot_cells(self.center)

            if true_mounting == NWS:
                origin = robot_cells[0]
            elif true_mounting == NS:
                origin = robot_cells[1]
            elif true_mounting == NES:
                origin = robot_cells[2]
            elif true_mounting == WS:
                origin = robot_cells[3]
            elif true_mounting == ES:
                origin = robot_cells[5]
            elif true_mounting == SWS:
                origin = robot_cells[6]
            elif true_mounting == SS:
                origin = robot_cells[7]
            elif true_mounting == SES:
                origin = robot_cells[8]
            elif true_mounting == CS:
                origin = robot_cells[4]

            y, x = get_matrix_coords(origin)
            read_range = list(range(sensor["blind_spot"] + 1, sensor["range"] + 1))

            for cell in read_range:
                try:
                    if true_facing == NORTH:
                        to_explore = (y + cell, x)
                    elif true_facing == EAST:
                        to_explore = (y, x + cell)
                    elif true_facing == SOUTH:
                        to_explore = (y - cell, x)
                    elif true_facing == WEST:
                        to_explore = (y, x - cell)

                    if to_explore[0] < 0 or to_explore[1] < 0:
                        print('ie')
                        raise IndexError

                    cell_index = get_grid_index(to_explore[0], to_explore[1])

                    # grid index, 0/1
                    updated_cell, value = self._mark_explored(cell_index)
                    if updated_cell is not None:
                        updated_cells[updated_cell] = value

                    if self.discovered_map[to_explore[0]][to_explore[1]] == 1:
                        raise IndexError

                except IndexError:
                    break

        return updated_cells

    def get_explore_string(self):
        """ Build and return the MDF string of the exploration status at the time of calling this function. """
        exploration_status = self.exploration_status[:]
        explore_str = ''.join(str(grid) for row in exploration_status for grid in row)
        explore_status_string = '11%s11' % explore_str
        explore_status_string = str(hex(int(explore_status_string, 2)))
        return explore_status_string[2:]

    def get_map_string(self):
        """ Build and return the MDF string of the robot's internal map at the time of calling this function. """
        discovered_map = self.discovered_map[:]
        map_str = ''.join(str(grid) for row in discovered_map for grid in row if grid != 2)
        pad_length = (4 - ((len(map_str) + 4) % 4)) % 4
        pad = '0' * pad_length
        map_string = '1111%s%s' % (map_str, pad)
        map_string = str(hex(int(map_string, 2)))
        map_string = map_string[3:]
        return map_string
